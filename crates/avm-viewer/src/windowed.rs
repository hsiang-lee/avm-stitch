use std::path::{Path, PathBuf};
use std::sync::Arc;

use avm_core::bowl::BowlModel;
use avm_core::calibration::Calibration;
use avm_render::{
    create_depth_view, create_instance, load_camera_image, request_device, ReadbackError,
    GpuSceneRenderer, RenderError, RenderSize, RenderTarget, SceneData, TextureReadback,
    BACKGROUND_CLEAR,
};
use thiserror::Error;
use winit::application::ApplicationHandler;
use winit::dpi::{LogicalSize, PhysicalPosition, PhysicalSize};
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::window::{Window, WindowId};

use crate::controller::{DragButtons, OrbitController};

/// Logical size of the interactive window before the window manager resizes it.
pub const WINDOW_WIDTH: u32 = 1280;
pub const WINDOW_HEIGHT: u32 = 720;
/// A wheel line is treated as this many pixels of vertical drag.
const WHEEL_LINE_PIXELS: f64 = 50.0;
const CLEAR: wgpu::Color = wgpu::Color {
    r: BACKGROUND_CLEAR[0],
    g: BACKGROUND_CLEAR[1],
    b: BACKGROUND_CLEAR[2],
    a: BACKGROUND_CLEAR[3],
};

#[derive(Debug, Error)]
pub enum ViewerError {
    #[error("no display available for the interactive window: {0}")]
    NoDisplay(String),
    #[error("the window system event loop failed: {0}")]
    EventLoop(String),
    #[error("failed to create the window: {0}")]
    Window(String),
    #[error("the window surface is not supported by this adapter")]
    SurfaceUnsupported,
    #[error("the window surface reported a validation error")]
    SurfaceValidation,
    #[error("the window surface cannot be copied to a screenshot on this adapter")]
    ScreenshotUnsupported,
    #[error("failed to write the screenshot: {0}")]
    Screenshot(String),
    #[error(transparent)]
    Image(#[from] avm_render::ImageError),
    #[error(transparent)]
    Render(#[from] RenderError),
}

impl From<ReadbackError> for ViewerError {
    fn from(error: ReadbackError) -> Self {
        Self::Screenshot(error.to_string())
    }
}

/// How the viewer ends: an optional frame budget makes it scriptable, and an
/// optional screenshot captures the first presented frame.
#[derive(Clone, Debug, Default)]
pub struct ViewerOptions {
    pub frames: Option<u32>,
    pub screenshot: Option<PathBuf>,
}

pub fn run_windowed(
    data_dir: &Path,
    calibration: &Calibration,
    model: &BowlModel,
    options: &ViewerOptions,
) -> Result<(), ViewerError> {
    let images = calibration
        .image_files
        .iter()
        .map(|name| load_camera_image(data_dir.join(name)))
        .collect::<Result<Vec<_>, _>>()?;
    let scene = SceneData::from_bowl_model(model).with_camera_images(images);
    let controller = OrbitController::from_home(&calibration.bowl_params());

    let event_loop = EventLoop::new().map_err(|error| ViewerError::NoDisplay(error.to_string()))?;
    event_loop.set_control_flow(ControlFlow::Wait);

    let mut app = WindowApp {
        scene,
        controller,
        state: None,
        last_cursor: None,
        buttons: DragButtons::default(),
        error: None,
        frames: options.frames,
        frames_drawn: 0,
        screenshot: options.screenshot.clone(),
        screenshot_taken: false,
    };
    event_loop
        .run_app(&mut app)
        .map_err(|error| ViewerError::EventLoop(error.to_string()))?;
    if app.frames.is_some() {
        println!("Presented {} frame(s)", app.frames_drawn);
    }
    if let Some(error) = app.error {
        return Err(error);
    }
    verify_capture(options.screenshot.as_deref(), app.screenshot_taken)
}

/// A requested screenshot that was never written is a failure, not a quiet
/// success: the caller asked for a file and the run did not produce one.
fn verify_capture(requested: Option<&Path>, captured: bool) -> Result<(), ViewerError> {
    match (requested, captured) {
        (Some(path), false) => Err(ViewerError::Screenshot(format!(
            "no frame was captured to {}",
            path.display()
        ))),
        _ => Ok(()),
    }
}

/// What one redraw actually did. A surface that is not acquired has neither
/// presented nor captured anything, so counting it as a frame would let
/// `--frames N` exit before the image exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FrameOutcome {
    presented: bool,
    screenshot: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameStep {
    Exit,
    Redraw,
    Idle,
}

/// Folds one redraw into the frame budget: only a presented frame counts, and a
/// budgeted run keeps asking for redraws until the budget is met.
fn next_step(outcome: FrameOutcome, frames_drawn: u32, limit: Option<u32>) -> (u32, FrameStep) {
    let drawn = frames_drawn + u32::from(outcome.presented);
    let step = match limit {
        Some(limit) if drawn >= limit => FrameStep::Exit,
        Some(_) => FrameStep::Redraw,
        None => FrameStep::Idle,
    };
    (drawn, step)
}

fn set_button(buttons: &mut DragButtons, button: MouseButton, pressed: bool) {
    match button {
        MouseButton::Left => buttons.left = pressed,
        MouseButton::Right => buttons.right = pressed,
        MouseButton::Middle => buttons.middle = pressed,
        _ => {}
    }
}

struct RenderState {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    renderer: GpuSceneRenderer,
    depth: wgpu::TextureView,
}

impl RenderState {
    fn render_frame(
        &self,
        controller: &OrbitController,
        screenshot: Option<&Path>,
    ) -> Result<FrameOutcome, ViewerError> {
        match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => {
                let view = frame
                    .texture
                    .create_view(&wgpu::TextureViewDescriptor::default());
                let mut encoder =
                    self.device
                        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                            label: Some("avm-window-encoder"),
                        });
                let aspect = self.config.width as f32 / self.config.height as f32;
                self.renderer.render(
                    &self.queue,
                    &mut encoder,
                    RenderTarget {
                        color: &view,
                        depth: &self.depth,
                    },
                    &controller.view_spec(aspect),
                    CLEAR,
                );
                self.queue.submit([encoder.finish()]);
                if let Some(path) = screenshot {
                    capture_surface(
                        &self.device,
                        &self.queue,
                        &frame.texture,
                        RenderSize::new(self.config.width, self.config.height),
                        self.config.format,
                        path,
                    )?;
                }
                self.queue.present(frame);
                Ok(FrameOutcome {
                    presented: true,
                    screenshot: screenshot.is_some(),
                })
            }
            wgpu::CurrentSurfaceTexture::Lost | wgpu::CurrentSurfaceTexture::Outdated => {
                self.surface.configure(&self.device, &self.config);
                self.window.request_redraw();
                Ok(FrameOutcome {
                    presented: false,
                    screenshot: false,
                })
            }
            // wgpu does not expose the validation message here and the viewer
            // cannot recover from it, so it ends the run with an error rather
            // than reporting a silent success.
            wgpu::CurrentSurfaceTexture::Validation => Err(ViewerError::SurfaceValidation),
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                Ok(FrameOutcome {
                    presented: false,
                    screenshot: false,
                })
            }
        }
    }

    /// Applies a new surface size: one place that knows the configuration and
    /// the depth target must both follow it.
    fn resize(&mut self, size: PhysicalSize<u32>) {
        if size.width == 0 || size.height == 0 {
            return;
        }
        self.config.width = size.width;
        self.config.height = size.height;
        self.surface.configure(&self.device, &self.config);
        self.depth = create_depth_view(&self.device, RenderSize::new(size.width, size.height));
        self.window.request_redraw();
    }
}

struct WindowApp {
    scene: SceneData,
    controller: OrbitController,
    state: Option<RenderState>,
    last_cursor: Option<(f64, f64)>,
    buttons: DragButtons,
    error: Option<ViewerError>,
    frames: Option<u32>,
    frames_drawn: u32,
    screenshot: Option<PathBuf>,
    screenshot_taken: bool,
}

impl WindowApp {
    fn create_state(&self, event_loop: &ActiveEventLoop) -> Result<RenderState, ViewerError> {
        let (window, instance, surface) = create_window_and_surface(event_loop)?;
        let (adapter, device, queue) =
            request_device(&instance, Some(&surface), "avm-window-device")?;
        let (config, format) = configure_surface(
            &window,
            &surface,
            &adapter,
            &device,
            self.screenshot.is_some(),
        )?;

        let renderer = GpuSceneRenderer::new(&device, &queue, &self.scene, format)?;
        let depth = create_depth_view(&device, RenderSize::new(config.width, config.height));
        Ok(RenderState {
            window,
            surface,
            device,
            queue,
            config,
            renderer,
            depth,
        })
    }

    fn handle_mouse_input(&mut self, button_state: ElementState, button: MouseButton) {
        set_button(
            &mut self.buttons,
            button,
            button_state == ElementState::Pressed,
        );
        if !self.buttons.any() {
            self.last_cursor = None;
        }
    }

    fn handle_cursor_moved(&mut self, state: &mut RenderState, position: PhysicalPosition<f64>) {
        let now = (position.x, position.y);
        let delta = self
            .last_cursor
            .map(|last| (now.0 - last.0, now.1 - last.1));
        self.last_cursor = Some(now);
        let Some((dx, dy)) = delta else {
            return;
        };
        let Some(mode) = self.buttons.mode() else {
            return;
        };
        self.controller.drag(mode, dx, dy, state.config.height);
        state.window.request_redraw();
    }

    fn handle_mouse_wheel(&mut self, state: &mut RenderState, delta: MouseScrollDelta) {
        let dy = match delta {
            MouseScrollDelta::LineDelta(_, y) => f64::from(y) * WHEEL_LINE_PIXELS,
            MouseScrollDelta::PixelDelta(position) => position.y,
        };
        self.controller.zoom(dy);
        state.window.request_redraw();
    }

    fn handle_redraw(&mut self, event_loop: &ActiveEventLoop, state: &mut RenderState) {
        let screenshot = if self.screenshot_taken {
            None
        } else {
            self.screenshot.as_deref()
        };
        match state.render_frame(&self.controller, screenshot) {
            Ok(outcome) => {
                self.screenshot_taken |= outcome.screenshot;
                let (drawn, step) = next_step(outcome, self.frames_drawn, self.frames);
                self.frames_drawn = drawn;
                match step {
                    FrameStep::Exit => event_loop.exit(),
                    FrameStep::Redraw => state.window.request_redraw(),
                    FrameStep::Idle => {}
                }
            }
            Err(error) => {
                self.error = Some(error);
                event_loop.exit();
            }
        }
    }
}

fn create_window_and_surface(
    event_loop: &ActiveEventLoop,
) -> Result<(Arc<Window>, wgpu::Instance, wgpu::Surface<'static>), ViewerError> {
    let window = Arc::new(
        event_loop
            .create_window(
                Window::default_attributes()
                    .with_title("avm_viewer")
                    .with_inner_size(LogicalSize::new(
                        f64::from(WINDOW_WIDTH),
                        f64::from(WINDOW_HEIGHT),
                    )),
            )
            .map_err(|error| ViewerError::NoDisplay(format!("create window: {error}")))?,
    );
    let instance = create_instance();
    let surface = instance
        .create_surface(window.clone())
        .map_err(|error| ViewerError::Window(format!("create surface: {error}")))?;
    Ok((window, instance, surface))
}

/// Picks a surface format and configures the surface for the current size.
fn configure_surface(
    window: &Window,
    surface: &wgpu::Surface<'static>,
    adapter: &wgpu::Adapter,
    device: &wgpu::Device,
    want_copy: bool,
) -> Result<(wgpu::SurfaceConfiguration, wgpu::TextureFormat), ViewerError> {
    let capabilities = surface.get_capabilities(adapter);
    let format = capabilities
        .formats
        .iter()
        .copied()
        .find(|format| !format.is_srgb())
        .or_else(|| capabilities.formats.first().copied())
        .ok_or(ViewerError::SurfaceUnsupported)?;
    let mut usage = wgpu::TextureUsages::RENDER_ATTACHMENT;
    if want_copy {
        if !capabilities.usages.contains(wgpu::TextureUsages::COPY_SRC) {
            return Err(ViewerError::ScreenshotUnsupported);
        }
        usage |= wgpu::TextureUsages::COPY_SRC;
    }
    let size = window.inner_size();
    let config = wgpu::SurfaceConfiguration {
        usage,
        format,
        color_space: wgpu::SurfaceColorSpace::Auto,
        width: size.width.max(1),
        height: size.height.max(1),
        present_mode: wgpu::PresentMode::Fifo,
        desired_maximum_frame_latency: 2,
        alpha_mode: capabilities
            .alpha_modes
            .first()
            .copied()
            .unwrap_or(wgpu::CompositeAlphaMode::Auto),
        view_formats: Vec::new(),
    };
    surface.configure(device, &config);
    Ok((config, format))
}

impl ApplicationHandler for WindowApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        match self.create_state(event_loop) {
            Ok(state) => {
                state.window.request_redraw();
                self.state = Some(state);
            }
            Err(error) => {
                self.error = Some(error);
                event_loop.exit();
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        let Some(mut state) = self.state.take() else {
            return;
        };
        if window_id != state.window.id() {
            self.state = Some(state);
            return;
        }
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => state.resize(size),
            WindowEvent::MouseInput {
                state: button_state,
                button,
                ..
            } => self.handle_mouse_input(button_state, button),
            WindowEvent::CursorMoved { position, .. } => {
                self.handle_cursor_moved(&mut state, position)
            }
            WindowEvent::MouseWheel { delta, .. } => self.handle_mouse_wheel(&mut state, delta),
            WindowEvent::RedrawRequested => self.handle_redraw(event_loop, &mut state),
            _ => {}
        }
        self.state = Some(state);
    }
}

/// Copies the presented surface texture back and writes it as an RGBA PNG.
/// Fails loudly when the adapter cannot map the copy instead of skipping.
fn capture_surface(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
    size: RenderSize,
    format: wgpu::TextureFormat,
    path: &Path,
) -> Result<(), ViewerError> {
    let readback = TextureReadback::new(device, size);
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("avm-screenshot-encoder"),
    });
    readback.copy_from_texture(&mut encoder, texture);
    queue.submit([encoder.finish()]);
    let rgba = readback.read_opaque(device, format)?;

    let image = image::RgbaImage::from_raw(size.width, size.height, rgba)
        .ok_or_else(|| ViewerError::Screenshot("frame buffer size does not match".into()))?;
    image
        .save(path)
        .map_err(|error| ViewerError::Screenshot(error.to_string()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{next_step, verify_capture, FrameOutcome, FrameStep};

    const PRESENTED: FrameOutcome = FrameOutcome {
        presented: true,
        screenshot: true,
    };
    const SKIPPED: FrameOutcome = FrameOutcome {
        presented: false,
        screenshot: false,
    };

    /// `Lost`/`Outdated`/`Timeout`/`Occluded` return a skipped outcome: they
    /// must not spend frame budget or claim a screenshot.
    #[test]
    fn a_skipped_redraw_does_not_count_as_a_frame() {
        assert_eq!((0, FrameStep::Redraw), next_step(SKIPPED, 0, Some(2)));
        assert_eq!((1, FrameStep::Redraw), next_step(SKIPPED, 1, Some(2)));
    }

    #[test]
    fn a_presented_frame_spends_the_budget() {
        assert_eq!((1, FrameStep::Redraw), next_step(PRESENTED, 0, Some(2)));
        assert_eq!((2, FrameStep::Exit), next_step(PRESENTED, 1, Some(2)));
        assert_eq!((1, FrameStep::Exit), next_step(PRESENTED, 0, Some(1)));
    }

    #[test]
    fn an_interactive_run_never_self_exits() {
        assert_eq!((1, FrameStep::Idle), next_step(PRESENTED, 0, None));
        assert_eq!((0, FrameStep::Idle), next_step(SKIPPED, 0, None));
    }

    #[test]
    fn a_requested_but_missing_screenshot_fails_the_run() {
        let path = std::path::Path::new("/tmp/never-written.png");
        assert!(verify_capture(Some(path), false).is_err());
        assert!(verify_capture(Some(path), true).is_ok());
        assert!(verify_capture(None, false).is_ok());
    }
}
