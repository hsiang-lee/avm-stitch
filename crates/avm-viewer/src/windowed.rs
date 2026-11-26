//! winit + wgpu window that draws the same scene as the offscreen renderer.

use std::path::Path;
use std::sync::Arc;

use avm_core::bowl::BowlModel;
use avm_core::calibration::Calibration;
use avm_render::{
    create_depth_view, create_instance, load_camera_image, request_device, GpuSceneRenderer,
    RenderError, RenderSize, RenderTarget, SceneData,
};
use thiserror::Error;
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::window::{Window, WindowId};

use crate::controller::{DragButtons, OrbitController};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
/// A wheel line is treated as this many pixels of vertical drag.
const WHEEL_LINE_PIXELS: f64 = 50.0;
const CLEAR: wgpu::Color = wgpu::Color {
    r: 30.0 / 255.0,
    g: 36.0 / 255.0,
    b: 50.0 / 255.0,
    a: 1.0,
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
    #[error(transparent)]
    Image(#[from] avm_render::ImageError),
    #[error(transparent)]
    Render(#[from] RenderError),
}

pub fn run_windowed(
    data_dir: &Path,
    calibration: &Calibration,
    model: &BowlModel,
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
    };
    event_loop
        .run_app(&mut app)
        .map_err(|error| ViewerError::EventLoop(error.to_string()))?;
    match app.error {
        Some(error) => Err(error),
        None => Ok(()),
    }
}

/// Maps winit's button enum onto the three trackball buttons.
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
    fn render_frame(&self, controller: &OrbitController) -> Result<(), ViewerError> {
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
                self.queue.present(frame);
                Ok(())
            }
            wgpu::CurrentSurfaceTexture::Lost | wgpu::CurrentSurfaceTexture::Outdated => {
                self.surface.configure(&self.device, &self.config);
                self.window.request_redraw();
                Ok(())
            }
            // wgpu does not expose the validation message here and the viewer
            // cannot recover from it, so it ends the run with an error rather
            // than reporting a silent success.
            wgpu::CurrentSurfaceTexture::Validation => Err(ViewerError::SurfaceValidation),
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => Ok(()),
        }
    }
}

struct WindowApp {
    scene: SceneData,
    controller: OrbitController,
    state: Option<RenderState>,
    last_cursor: Option<(f64, f64)>,
    buttons: DragButtons,
    error: Option<ViewerError>,
}

impl WindowApp {
    fn create_state(&self, event_loop: &ActiveEventLoop) -> Result<RenderState, ViewerError> {
        let (window, instance, surface) = create_window_and_surface(event_loop)?;
        let (adapter, device, queue) =
            request_device(&instance, Some(&surface), "avm-window-device")?;
        let (config, format) = configure_surface(&window, &surface, &adapter, &device)?;

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
}

/// Creates the window, the instance that owns it and its surface.
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
) -> Result<(wgpu::SurfaceConfiguration, wgpu::TextureFormat), ViewerError> {
    let capabilities = surface.get_capabilities(adapter);
    let format = capabilities
        .formats
        .iter()
        .copied()
        .find(|format| !format.is_srgb())
        .or_else(|| capabilities.formats.first().copied())
        .ok_or(ViewerError::SurfaceUnsupported)?;
    let size = window.inner_size();
    let config = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
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
        let Some(state) = self.state.as_mut() else {
            return;
        };
        if window_id != state.window.id() {
            return;
        }
        let mut fatal_error = None;
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if size.width == 0 || size.height == 0 {
                    return;
                }
                state.config.width = size.width;
                state.config.height = size.height;
                state.surface.configure(&state.device, &state.config);
                state.depth =
                    create_depth_view(&state.device, RenderSize::new(size.width, size.height));
                state.window.request_redraw();
            }
            WindowEvent::MouseInput {
                state: button_state,
                button,
                ..
            } => {
                set_button(
                    &mut self.buttons,
                    button,
                    button_state == ElementState::Pressed,
                );
                if !self.buttons.any() {
                    self.last_cursor = None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
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
            WindowEvent::MouseWheel { delta, .. } => {
                let dy = match delta {
                    MouseScrollDelta::LineDelta(_, y) => f64::from(y) * WHEEL_LINE_PIXELS,
                    MouseScrollDelta::PixelDelta(position) => position.y,
                };
                self.controller.zoom(dy);
                state.window.request_redraw();
            }
            WindowEvent::RedrawRequested => {
                if let Err(error) = state.render_frame(&self.controller) {
                    fatal_error = Some(error);
                }
            }
            _ => {}
        }
        if let Some(error) = fatal_error {
            self.error = Some(error);
            event_loop.exit();
        }
    }
}
