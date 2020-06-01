
ifm3d - Implementing a Custom Image Container
=============================================

`ifm3d` provides a full software stack for writing computer vision / robotics
perception applications based upon ifm 3D cameras and the pmd ToF chip. `ifm3d`
ships, as part of its core, two *image modules*. They are `image` and
`opencv`. The `image` module bridges the 3D camera data to both OpenCV and
PCL, while `opencv` exclusively bridges to OpenCV dropping any required PCL
dependencies for users. `ifm3d`, through its modularity, also encourages the
creation of new image containers to be utilized within the overall `ifm3d`
ecosystem. For example, you may want to create bindings to
[Eigen](http://eigen.tuxfamily.org),
[xtensor](https://github.com/QuantStack/xtensor), or maybe even a proprietary
format to be used internally at your company. The purpose of this document is
to help would-be image-container authors to be able to start quickly.

The example code in this document is intentionally basic so as to not cloud the
mechanics of writing an image container with the practicalities of writing
robust real-world software. For real-world examples, please consult the source
code for the `image` and/or `opencv` image containers shipped as part of this
source distribution. We note, the `image` module provides an example which
builds a library that can be linked to at runtime, while the `opencv` module is
implemented as a header-only library.


The first step in writing an image container for `ifm3d` is to sub-class the
`ifm3d::ByteBuffer` class (part of the `framegrabber`
module). `ifm3d::ByteBuffer` is a template class designed to facilitate
*static* (compile-time) polymorphic class hierarchies. That is, there are no
`virtual` functions (i.e., no runtime polymorphism) and by association, no
vtable lookups at runtime. To those who are familiar with this idea,
`ifm3d::ByteBuffer` imposes the usage of
[CRTP](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern).

The most simple image container implementation looks something like this:

```c++
class MyBuff : public ifm3d::ByteBuffer<MyBuff>
{
public:
  MyBuff() : ifm3d::ByteBuffer<MyBuff>()
  {
    LOG(INFO) << "ctor";
  }

  template <typename T>
  void ImCreate(ifm3d::image_chunk im,
                std::uint32_t fmt,
                std::size_t idx,
                std::uint32_t width,
                std::uint32_t height,
                int nchan,
                std::uint32_t npts,
                const std::vector<std::uint8_t>& bytes)
  {
    LOG(INFO) << "ImCreate: " << (int) im;
  }

  template <typename T>
  void CloudCreate(std::uint32_t fmt,
                   std::size_t xidx,
                   std::size_t yidx,
                   std::size_t zidx,
                   std::uint32_t width,
                   std::uint32_t height,
                   std::uint32_t npts,
                   const std::vector<std::uint8_t>& bytes)
  {
    LOG(INFO) << "CloudCreate";
  }

};
```

There are two *callback* (template) functions that must be implemented by every
image container. They are ``ImCreate`` and ``CloudCreate``. The arguments are
well described in the `modules/framegrabber/include/ifm3d/fg/byte_buffer.h`
file and for the sake of brevity are not repeated here. However, we do want to
point out some key pieces of information critical for image container
authors. This information now follows.

For each frame received from the camera all 2D images will be processed first
by calls to ``ImCreate`` (called n-times per frame, once for each image type,
depending upon your active PCIC schema passed to the framegrabber). Then, a
single call to ``CloudCreate`` is made with the 3D cartesian data *if* the
cartesian data are specified in your pcic schema. If they are not,
``CloudCreate`` will not be called. It is also important to note that the first
call to ``ImCreate`` for each frame will be the confidence data. This is so
that for each subsequent image type (or point cloud), the confidence image (for
the current frame) can be referenced to see, for example, if a given pixel is
valid or not -- i.e., you may want to populate bad pixels as NaN in your
container implementation.

We also note that both ``ImCreate`` and ``CloudCreate`` are template functions
that take a single template parameter ``T``. ``T`` represents the C++ pixel
data type. So, if you are using a strongly-typed container that supports
template parameters, you can pass ``T`` straight through and not incur the
overhead of performing a lookup on the ``fmt`` argument (which is redundant
with ``T``).

Using your custom image container will look very similar to using the standard
image containers shipped with ``ifm3d``. Synthesizing a hypothetical example
that would utilize our (very simple and useless) container from above, you
could write client code like:

```c++

auto cam = ifm3d::Camera::MakeShared();
auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
auto buff = std::make_shared<MyBuff>();

while (fg->WaitForFrame(buff.get(), 1000))
 {
    // ... now use buff ..
 }
```

For clarity, we also note that the methods/accessors defined in the base
`ifm3d::ByteBuffer` class are available to your custom containers as well. So,
access to the extrinsics, timestamp, illumination temperature, and raw bytes
are available with no extra work required by you.

Again, we strongly urge you to inspect the code for the `image` and `opencv`
modules that we ship with `ifm3d`. This will give you a very concrete example
as to how you can implement your own image container types in a more realistic
way (i.e., how to parse the pixels, byte swap them, deal with copy/move
semantics, etc.). If you have questions, feel free to ask on our
[issue tracker](https://github.com/ifm/ifm3d/issues).
