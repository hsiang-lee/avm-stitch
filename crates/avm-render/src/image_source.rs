//! `image` is the only image IO dependency.

use std::path::Path;

use thiserror::Error;

#[derive(Debug, Error)]
pub enum ImageError {
    #[error("failed to load camera image {path}: {source}")]
    Load {
        path: String,
        #[source]
        source: image::ImageError,
    },
}

/// Load a camera image as RGBA8, the format the camera texture array expects.
pub fn load_camera_image(path: impl AsRef<Path>) -> Result<image::RgbaImage, ImageError> {
    let path = path.as_ref();
    let image = image::open(path).map_err(|source| ImageError::Load {
        path: path.display().to_string(),
        source,
    })?;
    Ok(image.to_rgba8())
}
