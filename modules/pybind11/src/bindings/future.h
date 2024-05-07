/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FUTURE
#define IFM3D_PYBIND_BINDING_FUTURE

#include <pybind11/pybind11.h>
#include <fmt/format.h>

class StopIteration : public py::stop_iteration
{
public:
  StopIteration(const py::object& result)
    : stop_iteration("--"),
      result_(std::move(result)) {};

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

template <>
class FutureAwaitable<void>
{
public:
  FutureAwaitable() : future_() {}

  FutureAwaitable(const std::shared_future<void>& future)
    : future_(std::move(future))
  {}

  FutureAwaitable<void>*
  iter()
  {
    return this;
  }

  FutureAwaitable<void>*
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
        throw StopIteration(py::none());
      }
  };

  void
  wait()
  {
    py::gil_scoped_release release;
    this->future_.wait();
    return this->future_.get();
  }

  std::tuple<bool, std::nullopt_t>
  wait_for(uint64_t timeout_ms)
  {
    py::gil_scoped_release release;

    if (this->future_.wait_for(std::chrono::milliseconds(timeout_ms)) !=
        std::future_status::ready)
      {
        return {false, std::nullopt};
      }

    return {true, std::nullopt};
  }

private:
  std::shared_future<void> future_;
};

template <typename T>
void
bind_future(py::module_& m,
            const char* name,
            const char* message,
            const char* result_type)
{
  // clang-format off

  py::class_<FutureAwaitable<T>> future(m, name, message);

  future.def(py::init<>(), message);
  future.def("__next__", &FutureAwaitable<T>::next);

  py::options options;
  options.disable_function_signatures();

  future.def("__iter__", &FutureAwaitable<T>::iter, py::doc(fmt::format("__iter__(self) -> typing.Generator[{0},{0},{0}]", result_type).c_str()));
  future.def("__await__", &FutureAwaitable<T>::await, py::doc(fmt::format("__await__(self) -> typing.Generator[{0},{0},{0}]", result_type).c_str()));

  future.def(
    "wait", 
    &FutureAwaitable<T>::wait, 
     py::doc(fmt::format(R"(
      wait(self) -> {}


      Blocks until the result becomes available.
    )", result_type).c_str()));

  future.def(
    "wait_for",
    &FutureAwaitable<T>::wait_for,
    py::arg("timeout_ms"),
    py::doc(fmt::format(R"(
      wait_for(self, timeout_ms: int) -> Tuple[bool, {}]


      Blocks until specified timeout runs out or the result to becomes available. 

      :return: a tuple (True, Result) if a result was received within the timeout, (False, None) otherwise.
    )", result_type).c_str()));

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FUTURE