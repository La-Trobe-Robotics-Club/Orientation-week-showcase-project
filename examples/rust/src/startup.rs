use opencv::videoio::{self, VideoCapture, VideoCaptureTrait};
use opencv::Result;

use opencv::highgui::*;

use crate::*;

pub fn setup_video_capture() -> VideoCapture {
    // create handle to video stream, using CAP_V4L2 to avoid annoying gstreamer startup warning
    // in actual use we'll probably want videoio::CAP_ANY
    let mut vid_cap = VideoCapture::new(0, videoio::CAP_V4L2).unwrap();

    // set webcam settings
    // matching what is reported by v4l2-ctl --all
    vid_cap.set(videoio::CAP_PROP_FPS, 30.).unwrap();
    vid_cap.set(videoio::CAP_PROP_FRAME_WIDTH, 640.).unwrap();
    vid_cap.set(videoio::CAP_PROP_FRAME_HEIGHT, 480.).unwrap();

    // improving visibility, note that 128 is default for
    // contract, brightness and saturation
    unsafe {
        vid_cap.set(videoio::CAP_PROP_AUTOFOCUS, 0.).unwrap(); // IMPORANT, changing focus = bad
        vid_cap.set(videoio::CAP_PROP_FOCUS, FOCUS).unwrap();
        vid_cap.set(videoio::CAP_PROP_CONTRAST, CONTRAST).unwrap();
        vid_cap
            .set(videoio::CAP_PROP_BRIGHTNESS, BRIGHTNESS)
            .unwrap();
        vid_cap
            .set(videoio::CAP_PROP_SATURATION, SATURATION)
            .unwrap();
        vid_cap.set(videoio::CAP_PROP_CONVERT_RGB, 1.).unwrap(); // pretty sure this is on by default
        vid_cap.set(videoio::CAP_PROP_EXPOSURE, EXPOSURE).unwrap(); // IMPORTANT the default is bad
    }

    vid_cap
}

pub fn create_windows() -> Result<()> {
    named_window("Raw", 0)?;
    named_window("Mask", 0)?;
    named_window("Output", 0)
}

pub fn create_trackbars() -> Result<()> {
    unsafe {
        create_trackbar(
            "Min H",
            "Mask",
            Some(&mut (MIN_HSV[0].clone() as i32)),
            179,
            TrackbarCallback::Some(Box::new(|v| {
                MIN_HSV[0] = v as u8;
            })),
        )?;
        create_trackbar(
            "Min S",
            "Mask",
            Some(&mut (MIN_HSV[1].clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                MIN_HSV[1] = v as u8;
            })),
        )?;
        create_trackbar(
            "Min V",
            "Mask",
            Some(&mut (MIN_HSV[2].clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                MIN_HSV[2] = v as u8;
            })),
        )?;
        create_trackbar(
            "Max H",
            "Mask",
            Some(&mut (MAX_HSV[0].clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                MAX_HSV[0] = v as u8;
            })),
        )?;
        create_trackbar(
            "Max S",
            "Mask",
            Some(&mut (MAX_HSV[1].clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                MAX_HSV[1] = v as u8;
            })),
        )?;
        create_trackbar(
            "Max V",
            "Mask",
            Some(&mut (MAX_HSV[2].clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                MAX_HSV[2] = v as u8;
            })),
        )?;
        create_trackbar(
            "Min Circle Radius",
            "Output",
            Some(&mut MIN_RAD.clone()),
            1024,
            TrackbarCallback::Some(Box::new(|v| {
                MIN_RAD = v;
            })),
        )?;
        create_trackbar(
            "Max Circle Radius",
            "Output",
            Some(&mut MAX_RAD.clone()),
            1023,
            TrackbarCallback::Some(Box::new(|v| {
                MAX_RAD = v;
            })),
        )?;

        create_trackbar(
            "Circle Parameter 1",
            "Output",
            Some(&mut CIRCLES_PARAM_1.clone()),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                CIRCLES_PARAM_1 = v;
            })),
        )?;
        create_trackbar(
            "Circle Parameter 2",
            "Output",
            Some(&mut CIRCLES_PARAM_2.clone()),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                CIRCLES_PARAM_2 = v;
            })),
        )?;
        set_trackbar_min("Circle Parameter 1", "Output", 1)?;
        set_trackbar_min("Circle Parameter 2", "Output", 1)?;

        create_trackbar(
            "Contrast",
            "Raw",
            Some(&mut (CONTRAST.clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                CONTRAST = v as f64;
            })),
        )?;
        create_trackbar(
            "Focus",
            "Raw",
            Some(&mut (FOCUS.clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                FOCUS = v as f64;
            })),
        )?;
        create_trackbar(
            "Saturation",
            "Raw",
            Some(&mut (SATURATION.clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                SATURATION = v as f64;
            })),
        )?;
        create_trackbar(
            "Brightness",
            "Raw",
            Some(&mut (BRIGHTNESS.clone() as i32)),
            255,
            TrackbarCallback::Some(Box::new(|v| {
                BRIGHTNESS = v as f64;
            })),
        )?;
        create_trackbar(
            "Exposure",
            "Raw",
            Some(&mut (EXPOSURE.clone() as i32)),
            1023,
            TrackbarCallback::Some(Box::new(|v| {
                EXPOSURE = v as f64;
            })),
        )?;
    }
    Ok(())
}
