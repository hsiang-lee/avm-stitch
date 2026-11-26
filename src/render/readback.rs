//! CPU readback of a rendered texture as tightly packed RGBA8 rows.

use thiserror::Error;

use crate::render::renderer::RenderSize;

/// Failure while copying a texture back to CPU memory.
#[derive(Debug, Error)]
#[error("{0}")]
pub struct ReadbackError(String);

impl ReadbackError {
    fn new(error: impl std::fmt::Display) -> Self {
        Self(error.to_string())
    }
}

/// A staging buffer sized for one `size` texture, carrying the padded row
/// layout wgpu requires for `copy_texture_to_buffer` plus the unpad on read.
pub struct TextureReadback {
    buffer: wgpu::Buffer,
    padded_bytes_per_row: u32,
    size: RenderSize,
}

impl TextureReadback {
    pub fn new(device: &wgpu::Device, size: RenderSize) -> Self {
        let unpadded = size.width * 4;
        let padded_bytes_per_row =
            unpadded.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
                * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("avm-readback"),
            size: u64::from(padded_bytes_per_row) * u64::from(size.height),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        Self {
            buffer,
            padded_bytes_per_row,
            size,
        }
    }

    /// Appends the copy of `texture` into this readback buffer to `encoder`.
    pub fn copy_from_texture(&self, encoder: &mut wgpu::CommandEncoder, texture: &wgpu::Texture) {
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &self.buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(self.padded_bytes_per_row),
                    rows_per_image: Some(self.size.height),
                },
            },
            wgpu::Extent3d {
                width: self.size.width,
                height: self.size.height,
                depth_or_array_layers: 1,
            },
        );
    }

    /// Maps the copied texture and returns its pixels in RGBA channel order,
    /// keeping the source alpha channel.
    pub fn read(
        &self,
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
    ) -> Result<Vec<u8>, ReadbackError> {
        self.read_with_alpha(device, format, false)
    }

    /// Like [`Self::read`], but forces an opaque alpha channel, which is what a
    /// presented surface needs for a screenshot.
    pub fn read_opaque(
        &self,
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
    ) -> Result<Vec<u8>, ReadbackError> {
        self.read_with_alpha(device, format, true)
    }

    fn read_with_alpha(
        &self,
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        opaque: bool,
    ) -> Result<Vec<u8>, ReadbackError> {
        let (sender, receiver) = std::sync::mpsc::channel();
        self.buffer
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = sender.send(result);
            });
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(ReadbackError::new)?;
        receiver
            .recv()
            .map_err(ReadbackError::new)?
            .map_err(ReadbackError::new)?;

        // The memory slots are the format's channel order, not channel names:
        // BGRA stores blue first, RGBA stores red first.
        let bgra = matches!(
            format,
            wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
        );
        let mut rgba = Vec::with_capacity((self.size.width * self.size.height * 4) as usize);
        {
            let mapped = self
                .buffer
                .slice(..)
                .get_mapped_range()
                .map_err(ReadbackError::new)?;
            let row_bytes = (self.size.width * 4) as usize;
            for row in 0..self.size.height as usize {
                let start = row * self.padded_bytes_per_row as usize;
                for pixel in mapped[start..start + row_bytes].chunks_exact(4) {
                    let [slot0, slot1, slot2, slot3] =
                        [pixel[0], pixel[1], pixel[2], pixel[3]];
                    let (r, g, b, a) = if bgra {
                        (slot2, slot1, slot0, slot3)
                    } else {
                        (slot0, slot1, slot2, slot3)
                    };
                    rgba.extend_from_slice(&[r, g, b, if opaque { 255 } else { a }]);
                }
            }
        }
        self.buffer.unmap();
        Ok(rgba)
    }
}
