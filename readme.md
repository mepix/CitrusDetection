# Orange Detection

This is my final project for EE243: Advanced Computer Vision

![Lidar on Laptop](./img/IMG_1011.png)

## Environment Setup

### Dependencies

Dependencies are installed using `brew`:

```sh
# Intel Realsense Camera API
brew install librealsense

# OpenCV Library
brew install opencv
```

After installing the dependencies, the location of the linked libraries can be found:

```sh
pkg-config --cflags --libs realsense2
pkg-config --cflags --libs opencv
```

### XCode Configuration

After the dependencies are installed, Xcode needs to be told where to find the libraries. This can be done by going to `Build Settings` then  editing the following parameters:

- Set Header Search Paths
- Set Library Search Paths
- Set Other Linker Flags

**Note:** packages installed by `brew` may not meet the requirements for a signed library as defined by Apple. This requirement can be overridden by going to: `Build Settings` then `Signing & Validation` and checking `Disable Library Validation`.


## Algorithm Structure

### Image Preprocessing

TODO: comments about reading in the image and frame alignment

### Detecting Citrus

## References

### Intel Realsense Library

- [Intel API Overview](https://github.com/IntelRealSense/librealsense/wiki/API-How-To)
- [Intel RS-Convert](https://github.com/IntelRealSense/librealsense/tree/master/tools/convert)
- [Intel Examples](https://github.com/IntelRealSense/librealsense/tree/master/examples)
- [Intel DOXYGEN](https://intelrealsense.github.io/librealsense/doxygen/index.html)
- [Advanced Stream Alignment](https://dev.intelrealsense.com/docs/rs-align-advanced)

### General

- [Setting up OpenCV Development with Xcode](https://medium.com/@jaskaranvirdi/setting-up-opencv-and-c-development-environment-in-xcode-b6027728003)
