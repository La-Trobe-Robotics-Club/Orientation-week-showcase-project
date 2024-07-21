use clap::Parser;
use opencv::{
    core::{self, Mat, Vec3f},
    highgui, imgproc,
    videoio::{self, VideoCapture, VideoCaptureTrait},
};
use std::time::Instant;

mod serial;
mod startup;

pub static mut MIN_HSV: [u8; 3] = [5, 80, 100];
pub static mut MAX_HSV: [u8; 3] = [90, 255, 255];

pub static mut MIN_RAD: i32 = 20;
pub static mut MAX_RAD: i32 = 40;
pub static mut CIRCLES_PARAM_1: i32 = 100;
pub static mut CIRCLES_PARAM_2: i32 = 30;

pub static mut CONTRAST: f64 = 128.;
pub static mut BRIGHTNESS: f64 = 128.;
pub static mut EXPOSURE: f64 = 511.;
pub static mut SATURATION: f64 = 128.;
pub static mut FOCUS: f64 = 5.0;

#[derive(Parser, Debug)]
#[command(version, about)]
pub struct Args {
    /// Wait for keypress before processing next frame when using the GUI
    #[arg(short, long, default_value_t = false)]
    wait: bool,
    /// Show GUI?
    #[arg(short, long, default_value_t = true)]
    gui: bool,
    /// location of the serial connection to the arduino (COM1, COM2 ect on windows)
    #[arg(short, long, default_value_t = String::from("/dev/ttyUSB0"))]
    serial: String,
    /// baud rate of the serial connection
    #[arg(short, long, default_value_t = 57600)]
    baud_rate: u32,
}

fn main() {
    env_logger::init();

    let args = Args::parse();

    let mut vid_cap = startup::setup_video_capture();
    let mut raw_frame = Mat::default();
    let mut mask_frame = Mat::default();
    let mut blurred_masked_frame = Mat::default();

    // get image dimensions and read first frame (to avoid skewing data later)
    let now = std::time::Instant::now();
    if let Err(e) = vid_cap.read(&mut raw_frame) {
        log::error!("Failed to read first frame: {e}.");
        return;
    }
    log::info!("{}ms for first read", now.elapsed().as_millis());
    if args.gui {
        if let Err(e) = startup::create_windows() {
            log::error!("Failed to create windows: {e}.");
            return;
        }
        if let Err(e) = startup::create_trackbars() {
            log::error!("Failed to create trackbars: {e}.");
            return;
        }
    }

    loop {
        let now = Instant::now();

        // read frame from camera
        if !read_into_frame(&mut vid_cap, &mut raw_frame) {
            log::warn!("Failed to read frame or no frames read!");
            continue;
        }

        // mask frame using HSV bounds
        if let Err(e) = mask_raw_frame(&mut raw_frame, &mut mask_frame) {
            log::warn!("Failed to mask frame: {e}.");
            continue;
        };

        // blur to reduce noise
        if let Err(e) = blur_frame(&mask_frame, &mut blurred_masked_frame) {
            log::warn!("Failed to blur frame: {e}.");
            continue;
        };

        // detect circles
        let circles = detect_cirlces(&mut blurred_masked_frame);

        log::debug!(
            "{} circles detected - total frame time: {}ms",
            circles.len(),
            now.elapsed().as_millis(),
        );

        // draw circles for the sake of visualisation
        if args.gui {
            for circle in &circles {
                let centre = core::Point2i::new(circle[0] as i32, circle[1] as i32);
                let _ = opencv::imgproc::circle(
                    &mut raw_frame,
                    centre,
                    circle[2] as i32,
                    core::Scalar_::new(0.0, 0.0, 255.0, 255.0),
                    3,
                    opencv::imgproc::LINE_AA,
                    0,
                );
            }
        }

        // display GUI
        if args.gui {
            highgui::imshow("Raw", &raw_frame).unwrap();
            highgui::imshow("Mask", &mask_frame).unwrap();
            highgui::imshow("Output", &blurred_masked_frame).unwrap();

            if args.wait {
                let _ = opencv::highgui::wait_key(0);
            } else {
                let _ = opencv::highgui::poll_key();
            }
        }
        update_capture(&mut vid_cap);
    }
}

fn read_into_frame(vid_cap: &mut VideoCapture, raw_frame: &mut Mat) -> bool {
    let now = Instant::now();
    let read_status = vid_cap.read(raw_frame);
    log::trace!("Frame read in {}ms", now.elapsed().as_millis());

    match read_status {
        Ok(false) | Err(_) => false,
        _ => true,
    }
}

fn mask_raw_frame(in_frame: &mut Mat, out_frame: &mut Mat) -> opencv::Result<()> {
    let mut tmp_mat = Mat::default();

    // convert to HSV
    let now = Instant::now();
    opencv::imgproc::cvt_color(
        in_frame,
        &mut tmp_mat,
        imgproc::ColorConversionCodes::COLOR_BGR2HSV as i32,
        0,
    )?;
    log::trace!("Frame converted to HSV in {}ms", now.elapsed().as_millis());

    // mask HSV
    let now = std::time::Instant::now();
    unsafe {
        opencv::core::in_range(
            &tmp_mat,
            &*std::ptr::addr_of!(MIN_HSV),
            &*std::ptr::addr_of!(MAX_HSV),
            out_frame,
        )?;
    }
    log::trace!("Frame masked in {}ms", now.elapsed().as_millis());

    Ok(())
}

fn blur_frame(in_frame: &Mat, out_frame: &mut Mat) -> opencv::Result<()> {
    let now = Instant::now();
    opencv::imgproc::median_blur(in_frame, out_frame, 5)?;
    log::trace!("Frame blurred in {}ms", now.elapsed().as_millis(),);
    Ok(())
}

fn detect_cirlces(in_frame: &Mat) -> core::Vector<Vec3f> {
    let mut circles: core::Vector<Vec3f> = core::Vector::new();
    let now = std::time::Instant::now();
    unsafe {
        let _ = opencv::imgproc::hough_circles(
            &in_frame,
            &mut circles,
            opencv::imgproc::HOUGH_GRADIENT,
            1.0,
            100.0,
            CIRCLES_PARAM_1 as f64,
            CIRCLES_PARAM_2 as f64,
            MIN_RAD,
            MAX_RAD,
        );
    }
    log::trace!("Circles detected in {}ms", now.elapsed().as_millis(),);
    circles
}

fn update_capture(vid_cap: &mut VideoCapture) {
    unsafe {
        vid_cap.set(videoio::CAP_PROP_FOCUS, FOCUS).unwrap();
        vid_cap.set(videoio::CAP_PROP_CONTRAST, CONTRAST).unwrap();
        vid_cap
            .set(videoio::CAP_PROP_BRIGHTNESS, BRIGHTNESS)
            .unwrap();
        vid_cap
            .set(videoio::CAP_PROP_SATURATION, SATURATION)
            .unwrap();
        vid_cap.set(videoio::CAP_PROP_EXPOSURE, EXPOSURE).unwrap();
    }
}
