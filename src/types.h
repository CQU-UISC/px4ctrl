#pragma once
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace px4ctrl {
using clock = std::chrono::high_resolution_clock;

// return time duration in milliseconds
inline double timeDuration(const clock::time_point &start,
                           const clock::time_point &end) {
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count());
}

// return time duration in milliseconds
inline double timePassed(const clock::time_point &start) {
  return timeDuration(start, clock::now());
}

// return time duration in seconds
inline double timePassedSeconds(const clock::time_point &start) {
  return timeDuration(start, clock::now()) / 1000.0f;
}

inline long to_uint64(const clock::time_point &time) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             time.time_since_epoch())
      .count();
}

inline clock::time_point from_uint64(const long &time) {
  return clock::time_point(std::chrono::milliseconds(time));
}

template <typename T> using Callback = std::function<void(const T &)>;
class Observer;
using FuncUnobserve = std::function<void(const Observer *)>;

class Observer {
public:
  Observer(FuncUnobserve func) : m_func(func) { return; }

  void unobserve() {
    m_func(this);
    return;
  };

  inline ~Observer() {
    unobserve();
    return;
  }

private:
  FuncUnobserve m_func;
};

/*
 * only create on Heap
 */
template <typename T> class Observable {
public:
  inline T value() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data;
  }

  inline void post(const T &data) {
    std::map<const Observer *, Callback<T>> snapshot;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_data = data;
      snapshot = m_callbacks;
    }
    for (auto &[_, cb] : snapshot) {
      cb(m_data);
    }
  }

  inline std::shared_ptr<Observer> observe(Callback<T> callback) {
    auto observer = std::make_shared<Observer>(
        std::bind(&Observable<T>::removeObserver, this, std::placeholders::_1));
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callbacks[observer.get()] = callback;
    return observer;
  }

private:
  friend class Observer;

  mutable std::mutex m_mutex;
  T m_data;
  std::map<const Observer *, Callback<T>> m_callbacks;
  inline void removeObserver(const Observer *observer) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callbacks.erase(const_cast<Observer *>(observer));
  }
};

template <typename T> using Px4Data = Observable<T>;

template <typename T> using Px4DataPtr = std::shared_ptr<Observable<T>>;

using Px4DataObserver = std::shared_ptr<Observer>;
} // namespace px4ctrl
