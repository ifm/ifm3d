# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] 

### Added
- O3RCamera::Schema method to get the current json schema

## 0.90.2 - 2021-09-16

- stlImage module (Image container based on STL)
- Removed copying of the tools header
- Example to upload docker container to O3R

## 0.90.1 - 2021-08-17

### Added
- Support for the new JSON based XML-RPC interface
- Support for 2D image data
- ifm3dpy_viewer python example
- Generate version based on last tag and commits since

### Changed
- Split Camera implementation into multiple classes
- IsO3D/IsO3X/IsO3R replaced by WhoAmI/AmI functions

## 0.90.0 - 2021-01-18

### Added

- Basic O3R support
- Initial IPv4 Discovery in the ifm3d command line tool

### Removed

- Hardcoded compiler Flags for Linux
- Copy of the header files during CMake build

