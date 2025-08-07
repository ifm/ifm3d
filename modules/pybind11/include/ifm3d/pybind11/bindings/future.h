/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FUTURE
#define IFM3D_PYBIND_BINDING_FUTURE

#include <fmt/format.h>
#include <future>
#include <pybind11/pybind11.h>
#include <utility>

namespace py = pybind11;

class StopIteration : public py::stop_iteration
{
public:
  StopIteration(py::object result)
    : stop_iteration("--"),
      _result(std::move(result)){};

  void
  set_error() const override
  {
    PyErr_SetObject(PyExc_StopIteration, this->_result.ptr());
  }

private:
  py::object _result;
};

template <typename RESULT_TYPE>
class FutureAwaitable
{
public:
  FutureAwaitable() : _future() {}

  FutureAwaitable(const std::shared_future<RESULT_TYPE>& future)
    : _future(std::move(future))
  {}

  FutureAwaitable<RESULT_TYPE>*
  Iter()
  {
    return this;
  }

  FutureAwaitable<RESULT_TYPE>*
  Await()
  {
    return this;
  }

  void
  Next()
  {
    // check if the future is resolved (with zero timeout)
    auto status = this->_future.wait_for(std::chrono::milliseconds(0));

    if (status == std::future_status::ready)
      {
        throw StopIteration(py::cast(_future.get()));
      }
  };

  RESULT_TYPE
  Wait()
  {
    py::gil_scoped_release release;
    this->_future.wait();
    return this->_future.get();
  }

  std::tuple<bool, std::optional<RESULT_TYPE>>
  WaitFor(uint64_t timeout_ms)
  {
    py::gil_scoped_release release;

    if (this->_future.wait_for(std::chrono::milliseconds(timeout_ms)) !=
        std::future_status::ready)
      {
        return {false, {}};
      }

    return {true, this->_future.get()};
  }

private:
  std::shared_future<RESULT_TYPE> _future;
};

template <>
class FutureAwaitable<void>
{
public:
  FutureAwaitable() = default;

  FutureAwaitable(const std::shared_future<void>& future) : _future(future) {}

  FutureAwaitable<void>*
  Iter()
  {
    return this;
  }

  FutureAwaitable<void>*
  Await()
  {
    return this;
  }

  void
  Next()
  {
    // check if the future is resolved (with zero timeout)
    auto status = this->_future.wait_for(std::chrono::milliseconds(0));

    if (status == std::future_status::ready)
      {
        throw StopIteration(py::none());
      }
  };

  void
  Wait()
  {
    py::gil_scoped_release release;
    this->_future.wait();
    this->_future.get();
  }

  std::tuple<bool, std::nullopt_t>
  WaitFor(uint64_t timeout_ms)
  {
    py::gil_scoped_release release;

    if (this->_future.wait_for(std::chrono::milliseconds(timeout_ms)) !=
        std::future_status::ready)
      {
        return {false, std::nullopt};
      }

    return {true, std::nullopt};
  }

private:
  std::shared_future<void> _future;
};

template <typename T>
void
bind_future(py::module_& m,
            const char* name,
            const char* message,
            const char* result_type)
{
  py::class_<FutureAwaitable<T>> future(m, name, message);

  future.def(py::init<>(), message);
  future.def("__next__", &FutureAwaitable<T>::Next);

  py::options options;
  options.disable_function_signatures();

  future.def(
    "__iter__",
    &FutureAwaitable<T>::Iter,
    py::doc(fmt::format("__iter__(self) -> typing.Generator[{0},{0},{0}]",
                        result_type)
              .c_str()));
  future.def(
    "__await__",
    &FutureAwaitable<T>::Await,
    py::doc(fmt::format("__await__(self) -> typing.Generator[{0},{0},{0}]",
                        result_type)
              .c_str()));

  future.def("wait",
             &FutureAwaitable<T>::Wait,
             py::doc(fmt::format(R"(
      wait(self) -> {}


      Blocks until the result becomes available.
    )",
                                 result_type)
                       .c_str()));

  future.def("wait_for",
             &FutureAwaitable<T>::WaitFor,
             py::arg("timeout_ms"),
             py::doc(fmt::format(R"(
      wait_for(self, timeout_ms: int) -> Tuple[bool, {}]


      Blocks until specified timeout runs out or the result to becomes available. 

      :return: a tuple (True, Result) if a result was received within the timeout, (False, None) otherwise.
    )",
                                 result_type)
                       .c_str()));
}

#endif // IFM3D_PYBIND_BINDING_FUTURE