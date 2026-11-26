//! GPU bring-up shared by the offscreen and window render paths.

use crate::offscreen::RenderError;

/// Creates a wgpu instance for a windowless or winit-owned surface. winit
/// supplies the display handle when the surface is created, so the instance
/// itself does not need one.
pub fn create_instance() -> wgpu::Instance {
    wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle())
}

/// Requests a low-power adapter and a device with downlevel defaults.
///
/// `label` names the device in wgpu validation messages. `compatible_surface`
/// is `Some` only for the window path, which must present to that surface; the
/// offscreen path passes `None`. The adapter is returned because the window
/// path asks it for the surface capabilities.
pub fn request_device(
    instance: &wgpu::Instance,
    compatible_surface: Option<&wgpu::Surface<'_>>,
    label: &str,
) -> Result<(wgpu::Adapter, wgpu::Device, wgpu::Queue), RenderError> {
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::LowPower,
        force_fallback_adapter: false,
        compatible_surface,
        apply_limit_buckets: false,
    }))
    .map_err(|_| RenderError::NoAdapter)?;

    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some(label),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::downlevel_defaults(),
        experimental_features: wgpu::ExperimentalFeatures::disabled(),
        memory_hints: wgpu::MemoryHints::default(),
        trace: wgpu::Trace::Off,
    }))
    .map_err(|error| RenderError::Device(error.to_string()))?;
    Ok((adapter, device, queue))
}
