use std::io::{Error, Write};

use glam::Vec3A;

const PC2_HEADER: &[u8; 12] = b"POINTCACHE2\0";

#[inline]
pub fn write_header<W: Write>(writer: &mut W, version: u32, population: u32, start_frame: f32, sample_rate: f32, total_samples: u32) -> Result<(), Error> {
    writer.write_all(PC2_HEADER)?;
    writer.write_all(&version.to_le_bytes())?;
    writer.write_all(&population.to_le_bytes())?;
    writer.write_all(&start_frame.to_le_bytes())?;
    writer.write_all(&sample_rate.to_le_bytes())?;
    writer.write_all(&total_samples.to_le_bytes())?;

    Ok(())
}

#[inline]
pub fn write_point<W: Write>(writer: &mut W, point: &Vec3A) -> Result<(), Error> {
    writer.write_all(&point.x.to_le_bytes())?;
    writer.write_all(&point.y.to_le_bytes())?;
    writer.write_all(&point.z.to_le_bytes())?;
    
    Ok(())
}