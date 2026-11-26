//! winit + wgpu window that draws the same scene as the offscreen renderer.

use std::path::Path;
use std::sync::Arc;

use avm_core::bowl::BowlModel;
use avm_core::calibration::Calibration;
use avm_render::{
    create_depth_view, load_camera_image, GpuSceneRenderer, RenderError, SceneData,
};
use thiserror::Error;
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::window::{Window, WindowId};

use crate::controller::{drag_mode, OrbitController};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
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
        buttons: Buttons::default(),
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

#[derive(Default, Clone, Copy)]
struct Buttons {
    left: bool,
    right: bool,
    middle: bool,
}

impl Buttons {
    fn any(&self) -> bool {
        self.left || self.right || self.middle
    }

    fn set(&mut self, button: MouseButton, pressed: bool) {
        match button {
            MouseButton::Left => self.left = pressed,
            MouseButton::Right => self.right = pressed,
            MouseButton::Middle => self.middle = pressed,
            _ => {}
        }
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

struct WindowApp {
    scene: SceneData,
    controller: OrbitController,
    state: Option<RenderState>,
    last_cursor: Option<(f64, f64)>,
    buttons: Buttons,
    error: Option<ViewerError>,
}

impl WindowApp {
    fn create_state(&self, event_loop: &ActiveEventLoop) -> Result<RenderState, ViewerError> {
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

        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let surface = instance
            .create_surface(window.clone())
            .map_err(|error| ViewerError::Window(format!("create surface: {error}")))?;
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            force_fallback_adapter: false,
            compatible_surface: Some(&surface),
            apply_limit_buckets: false,
        }))
        .map_err(|_| RenderError::NoAdapter)?;
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
                label: Some("avm-window-device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_defaults(),
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                memory_hints: wgpu::MemoryHints::default(),
                trace: wgpu::Trace::Off,
            }))
            .map_err(|error| RenderError::Device(error.to_string()))?;

        let capabilities = surface.get_capabilities(&adapter);
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
        surface.configure(&device, &config);

        let renderer = GpuSceneRenderer::new(&device, &queue, &self.scene, format)?;
        let depth = create_depth_view(&device, config.width, config.height);
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

    fn render_frame(
        state: &RenderState,
        controller: &OrbitController,
        event_loop: &ActiveEventLoop,
    ) {
        match state.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => {
                let view = frame
                    .texture
                    .create_view(&wgpu::TextureViewDescriptor::default());
                let mut encoder =
                    state
                        .device
                        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                            label: Some("avm-window-encoder"),
                        });
                let aspect = state.config.width as f32 / state.config.height as f32;
                state.renderer.render(
                    &state.queue,
                    &mut encoder,
                    &view,
                    &state.depth,
                    &controller.view_spec(aspect),
                    CLEAR,
                );
                state.queue.submit([encoder.finish()]);
                state.queue.present(frame);
            }
            wgpu::CurrentSurfaceTexture::Lost | wgpu::CurrentSurfaceTexture::Outdated => {
                state.surface.configure(&state.device, &state.config);
                state.window.request_redraw();
            }
            wgpu::CurrentSurfaceTexture::Validation => event_loop.exit(),
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {}
        }
    }
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
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if size.width == 0 || size.height == 0 {
                    return;
                }
                state.config.width = size.width;
                state.config.height = size.height;
                state.surface.configure(&state.device, &state.config);
                state.depth = create_depth_view(&state.device, size.width, size.height);
                state.window.request_redraw();
            }
            WindowEvent::MouseInput {
                state: button_state,
                button,
                ..
            } => {
                self.buttons
                    .set(button, button_state == ElementState::Pressed);
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
                let Some(mode) = drag_mode(
                    self.buttons.left,
                    self.buttons.right,
                    self.buttons.middle,
                ) else {
                    return;
                };
                self.controller.drag(mode, dx, dy, state.config.height);
                state.window.request_redraw();
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let dy = match delta {
                    MouseScrollDelta::LineDelta(_, y) => f64::from(y) * 50.0,
                    MouseScrollDelta::PixelDelta(position) => position.y,
                };
                self.controller.zoom(dy);
                state.window.request_redraw();
            }
            WindowEvent::RedrawRequested => {
                Self::render_frame(state, &self.controller, event_loop);
            }
            _ => {}
        }
    }
}
