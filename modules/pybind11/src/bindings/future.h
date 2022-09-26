/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FUTURE
#define IFM3D_PYBIND_BINDING_FUTURE

#include <pybind11/pybind11.h>

class StopIteration : public py::stop_iteration
{
public:
  StopIteration(const py::object& result)
    : stop_iteration("--"),
      result_(std::move(result)){};

  void
  set_error() const override
  {
    PyErr_SetObject(PyExc_StopIteration, this->result_.ptr());
  }

private:
  py::object result_;
};

template <typename ResultType>
class FutureAwaitable
{
public:
  FutureAwaitable() : future_() {}

  FutureAwaitable(const std::shared_future<ResultType>& future)
    : future_(std::move(future))
  {}

  FutureAwaitable<ResultType>*
  iter()
  {
    return this;
  }

  FutureAwaitable<ResultType>*
  await()
  {
    return this;
  }

  void
  next()
  {
    // check if the future is resolved (with zero timeout)
    auto status = this->future_.wait_for(std::chrono::milliseconds(0));

    if (status == std::future_status::ready)
      {
        throw StopIteration(py::cast(future_.get()));
      }
  };

  ResultType
  wait()
  {
    py::gil_scoped_release release;
    this->future_.wait();
    return this->future_.get();
  }

  std::tuple<bool, std::optional<ResultType>>
  wait_for(uint64_t timeout_ms)
  {
    py::gil_scoped_release release;

    if (this->future_.wait_for(std::chrono::milliseconds(timeout_ms)) !=
        std::future_status::ready)
      {
        return {false, {}};
      }

    return {true, this->future_.get()};
  }

private:
  std::shared_future<ResultType> future_;
};

template <typename T>
void
bind_future(py::module_& m, const char* name, const char* message)
{
  py::class_<FutureAwaitable<T>>(m, name, message)
    .def(py::init<>(), message)
    .def("__iter__", &FutureAwaitable<T>::iter)
    .def("__await__", &FutureAwaitable<T>::await)
    .def("__next__", &FutureAwaitable<T>::next)
    .def("wait",
         &FutureAwaitable<T>::wait,
         R"(
      Blocks until the frame becomes available.
    )")
    .def("wait_for",
         &FutureAwaitable<T>::wait_for,
         py::arg("timeout_ms"),
         R"(
      Waits for the frame to become available. Blocks until specified timeout
      has elapsed or the result becomes available, whichever comes first.
    )");
}

#endif // IFM3D_PYBIND_BINDING_FUTURE