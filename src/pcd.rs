use super::pointcloud::{FloatData, Point, PointCloud};
use anyhow::*;
use core::convert::TryInto;
use csv::{ReaderBuilder, StringRecord};
use itertools_num::*;
use num_traits::{Float, NumAssign};
use serde::Deserialize;
use std::fmt::Debug;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::str::FromStr;

#[derive(Default, Debug)]
struct PCDHeader {
    version: String,
    fields: Vec<String>,
    size: Vec<usize>,
    field_type: Vec<String>,
    count: Vec<usize>,
    width: usize,
    height: usize,
    data: String,
}

fn get_data_from_record<T>(record: &StringRecord, header: &PCDHeader) -> Result<T>
where
    T: Point + Default,
    <T as Point>::Item: FloatData + FromStr,
    <<T as Point>::Item as FromStr>::Err: Send + Debug + Into<Error>,
{
    let mut data = T::default();
    for (i, (field, field_type)) in header
        .fields
        .iter()
        .zip(header.field_type.iter())
        .enumerate()
    {
        if field_type != "F" {
            continue;
        }
        let field = data.point_field_mut(field.as_str());
        if let Ok(field_ok) = field {
            let res = record[i]
                .parse::<<T as Point>::Item>()
                .map_err(|e| anyhow!(e))?;
            if !res.is_nan() {
                *field_ok = res;
            }
        }
    }
    Ok(data)
}

fn get_data_from_binary<T>(buf_chunk: &[u8], header: &PCDHeader) -> Result<T>
where
    T: Point + Default,
    <T as Point>::Item: FloatData + NumAssign + Copy + for<'de> Deserialize<'de>,
{
    let mut data = T::default();
    let mut i_start = 0;
    let item_size = std::mem::size_of::<<T as Point>::Item>();
    for (field, field_type) in header.fields.iter().zip(header.field_type.iter()) {
        if field_type != "F" {
            continue;
        }
        let field = data.point_field_mut(field.as_str());
        if let Ok(field_ok) = field {
            let res: <T as Point>::Item = bincode::deserialize(&buf_chunk[i_start..(i_start + item_size)])?;
            if !res.is_nan() {
                *field_ok = res;
            }
        }
        i_start += item_size;
    }
    Ok(data)
}

pub fn read_pcd<T>(filename: &str) -> Result<PointCloud<T>>
where
    T: Point + Default + Debug,
    <T as Point>::Item: NumAssign + Copy + FromStr + for<'de> Deserialize<'de>,
    <<T as Point>::Item as FromStr>::Err: Send + Debug + Into<Error>,
{
    let mut pointcloud = PointCloud::<T>::new();
    let mut reader = BufReader::new(File::open(filename)?);
    let mut buf = String::new();

    let mut header = PCDHeader::default();
    while reader.read_line(&mut buf)? > 0 {
        buf.retain(|c| c != '\n');
        let is_header_end = {
            let v: Vec<&str> = buf.split(' ').collect();
            match v[0] {
                "VERSION" => header.version = v[1].to_string(),
                "FIELDS" => header.fields = v[1..].iter().map(|s| s.to_string()).collect(),
                "SIZE" => header.size = v[1..].iter().map(|s| s.parse().unwrap()).collect(),
                "TYPE" => header.field_type = v[1..].iter().map(|s| s.to_string()).collect(),
                "COUNT" => header.count = v[1..].iter().map(|s| s.parse().unwrap()).collect(),
                "WIDTH" => header.width = v[1].parse().unwrap(),
                "HEIGHT" => header.height = v[1].parse().unwrap(),
                "DATA" => header.data = v[1].to_string(),
                &_ => (),
            }
            v[0] == "DATA"
        };
        buf.clear();
        if is_header_end {
            break;
        }
    }

    let mut buf = Vec::new();
    reader.read_to_end(&mut buf)?;
    let field_sizes = header
        .count
        .iter()
        .zip(header.size.iter())
        .map(|(c, s)| c * s)
        .collect::<Vec<_>>();
    let field_offsets = [vec![0], field_sizes.iter().cumsum().collect::<Vec<usize>>()].concat();
    let fsize: usize = field_sizes.iter().sum();

    if header.data == "binary_compressed" {
        let uncompressed_size = u32::from_be_bytes(buf[4..8].try_into()?) as usize;
        let res_decomp =
            lzf::decompress(&buf[8..buf.len()], uncompressed_size).map_err(Error::msg)?;
        let offsets = [
            vec![0],
            field_sizes
                .iter()
                .map(|fs| fs * header.width * header.height)
                .cumsum()
                .collect::<Vec<usize>>(),
        ]
        .concat();

        buf.resize(uncompressed_size, 0);
        for i in 1..(header.width * header.height) {
            for (j, fs) in field_sizes.iter().enumerate() {
                let i_start = i * fsize + field_offsets[j];
                let i_end = i_start + fs;
                buf[i_start..i_end].copy_from_slice(
                    &res_decomp[(offsets[j] + i * fs)..(offsets[j] + (i + 1) * fs)],
                );
            }
        }
    }
    if header.data == "ascii" {
        let buf_str = String::from_utf8_lossy(&buf);
        let mut reader = ReaderBuilder::new()
            .delimiter(b' ')
            .from_reader(buf_str.as_bytes());
        for result in reader.records() {
            let record = result?;
            let data = get_data_from_record(&record, &header)?;
            pointcloud.add_data(data);
        }
    } else {
        for buf_chunk in buf.chunks(fsize) {
            let data = get_data_from_binary(buf_chunk, &header)?;
            pointcloud.add_data(data);
        }
    }
    Ok(pointcloud)
}
