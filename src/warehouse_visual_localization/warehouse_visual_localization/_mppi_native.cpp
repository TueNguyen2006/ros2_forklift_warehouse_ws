#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

struct RolloutConfig {
  double target_speed;
  int horizon_steps;
  int batch_size;
  double dt;
  double max_steer;
  double speed_noise_std;
  double steer_noise_std;
  double temperature;
  double path_weight;
  double heading_weight;
  double goal_weight;
  double control_weight;
  double smooth_weight;
  double progress_weight;
  double speed_weight;
  double wheelbase;
  double max_accel;
  double max_decel;
  double max_steer_rate;
};

struct SimState {
  double x;
  double y;
  double yaw;
  double v;
  double steer;
};

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

double clamp_abs(double value, double limit) {
  if (limit <= 0.0) {
    return 0.0;
  }
  return clamp(value, -limit, limit);
}

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<double> sequence_to_doubles(PyObject *obj, const char *name) {
  PyObject *seq = PySequence_Fast(obj, name);
  if (!seq) {
    throw std::runtime_error(name);
  }
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  std::vector<double> values;
  values.reserve(static_cast<size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    PyObject *item = PySequence_Fast_GET_ITEM(seq, i);
    const double value = PyFloat_AsDouble(item);
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      throw std::runtime_error(name);
    }
    values.push_back(value);
  }
  Py_DECREF(seq);
  return values;
}

double tuple_double(PyObject *args, Py_ssize_t index) {
  PyObject *item = PyTuple_GetItem(args, index);
  if (!item) {
    throw std::runtime_error("missing argument");
  }
  const double value = PyFloat_AsDouble(item);
  if (PyErr_Occurred()) {
    throw std::runtime_error("argument is not a float");
  }
  return value;
}

int tuple_int(PyObject *args, Py_ssize_t index) {
  PyObject *item = PyTuple_GetItem(args, index);
  if (!item) {
    throw std::runtime_error("missing argument");
  }
  const long value = PyLong_AsLong(item);
  if (PyErr_Occurred()) {
    throw std::runtime_error("argument is not an int");
  }
  return static_cast<int>(value);
}

unsigned int tuple_uint(PyObject *args, Py_ssize_t index) {
  PyObject *item = PyTuple_GetItem(args, index);
  if (!item) {
    throw std::runtime_error("missing argument");
  }
  const unsigned long value = PyLong_AsUnsignedLong(item);
  if (PyErr_Occurred()) {
    throw std::runtime_error("argument is not an unsigned int");
  }
  return static_cast<unsigned int>(value);
}

int nearest_index(double x, double y, const std::vector<double> &path_x,
                  const std::vector<double> &path_y) {
  int best_i = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_x.size(); ++i) {
    const double dx = x - path_x[i];
    const double dy = y - path_y[i];
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = static_cast<int>(i);
    }
  }
  return best_i;
}

void step_model(SimState &state, double cmd_v, double cmd_steer,
                const RolloutConfig &cfg) {
  const double dt = std::max(cfg.dt, 1e-6);
  const double steer_delta =
      clamp_abs(cmd_steer - state.steer, std::abs(cfg.max_steer_rate) * dt);
  state.steer = clamp(state.steer + steer_delta, -std::abs(cfg.max_steer),
                      std::abs(cfg.max_steer));

  const double accel_limit = (cmd_v >= state.v) ? cfg.max_accel : cfg.max_decel;
  const double accel = clamp_abs(cmd_v - state.v, accel_limit * dt) / dt;
  state.v = state.v + accel * dt;

  const double yaw_rate =
      (std::abs(state.steer) < 1e-5 || std::abs(state.v) < 1e-5)
          ? 0.0
          : -state.v * std::tan(state.steer) / std::max(cfg.wheelbase, 1e-6);
  state.yaw = wrap_angle(state.yaw + yaw_rate * dt);
  state.x += state.v * std::cos(state.yaw) * dt;
  state.y += state.v * std::sin(state.yaw) * dt;
}

double rollout_cost(const SimState &initial,
                    const std::vector<std::pair<double, double>> &sequence,
                    const std::vector<double> &path_x,
                    const std::vector<double> &path_y,
                    const std::vector<double> &path_yaw,
                    const RolloutConfig &cfg) {
  SimState sim = initial;
  double cost = 0.0;
  const int start_progress = nearest_index(sim.x, sim.y, path_x, path_y);
  double previous_steer = sim.steer;

  for (size_t k = 0; k < sequence.size(); ++k) {
    const double speed = sequence[k].first;
    const double steer = sequence[k].second;
    step_model(sim, speed, steer, cfg);
    const int nearest = nearest_index(sim.x, sim.y, path_x, path_y);
    const double dx = sim.x - path_x[nearest];
    const double dy = sim.y - path_y[nearest];
    const double path_error = std::sqrt(dx * dx + dy * dy);
    const double heading_error = std::abs(wrap_angle(path_yaw[nearest] - sim.yaw));
    const double step_scale = 1.0 + 0.04 * static_cast<double>(k);
    cost += step_scale *
            (cfg.path_weight * path_error * path_error +
             cfg.heading_weight * heading_error * heading_error +
             cfg.control_weight * steer * steer +
             cfg.smooth_weight * (steer - previous_steer) *
                 (steer - previous_steer) +
             cfg.speed_weight * (speed - cfg.target_speed) *
                 (speed - cfg.target_speed));
    previous_steer = steer;
  }

  const int end_progress = nearest_index(sim.x, sim.y, path_x, path_y);
  const double goal_dx = sim.x - path_x.back();
  const double goal_dy = sim.y - path_y.back();
  const double goal_error = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);
  cost += cfg.goal_weight * goal_error;
  cost -= cfg.progress_weight * std::max(0, end_progress - start_progress);
  return cost;
}

PyObject *mppi_step(PyObject *, PyObject *args) {
  try {
    if (PyTuple_Size(args) != 32) {
      PyErr_SetString(PyExc_TypeError, "mppi_step expects 32 positional arguments");
      return nullptr;
    }

    const SimState initial{tuple_double(args, 0), tuple_double(args, 1),
                           tuple_double(args, 2), tuple_double(args, 3),
                           tuple_double(args, 4)};

    auto path_x = sequence_to_doubles(PyTuple_GetItem(args, 5), "path_x");
    auto path_y = sequence_to_doubles(PyTuple_GetItem(args, 6), "path_y");
    auto path_yaw = sequence_to_doubles(PyTuple_GetItem(args, 7), "path_yaw");
    auto nominal_v = sequence_to_doubles(PyTuple_GetItem(args, 8), "nominal_v");
    auto nominal_steer =
        sequence_to_doubles(PyTuple_GetItem(args, 9), "nominal_steer");

    RolloutConfig cfg{};
    cfg.target_speed = tuple_double(args, 10);
    cfg.horizon_steps = std::max(2, tuple_int(args, 11));
    cfg.batch_size = std::max(1, tuple_int(args, 12));
    cfg.dt = tuple_double(args, 13);
    cfg.max_steer = tuple_double(args, 14);
    cfg.speed_noise_std = tuple_double(args, 15);
    cfg.steer_noise_std = tuple_double(args, 16);
    cfg.temperature = tuple_double(args, 17);
    cfg.path_weight = tuple_double(args, 18);
    cfg.heading_weight = tuple_double(args, 19);
    cfg.goal_weight = tuple_double(args, 20);
    cfg.control_weight = tuple_double(args, 21);
    cfg.smooth_weight = tuple_double(args, 22);
    cfg.progress_weight = tuple_double(args, 23);
    cfg.speed_weight = tuple_double(args, 24);
    cfg.wheelbase = tuple_double(args, 25);
    cfg.max_accel = tuple_double(args, 26);
    cfg.max_decel = tuple_double(args, 27);
    cfg.max_steer_rate = tuple_double(args, 28);
    const unsigned int seed = tuple_uint(args, 29);
    const double min_speed = tuple_double(args, 30);
    const double max_speed = tuple_double(args, 31);

    if (path_x.empty() || path_x.size() != path_y.size() ||
        path_x.size() != path_yaw.size()) {
      PyErr_SetString(PyExc_ValueError, "path arrays must be non-empty and equal length");
      return nullptr;
    }
    if (nominal_v.size() < static_cast<size_t>(cfg.horizon_steps) ||
        nominal_steer.size() < static_cast<size_t>(cfg.horizon_steps)) {
      PyErr_SetString(PyExc_ValueError, "nominal arrays shorter than horizon");
      return nullptr;
    }

    std::mt19937 rng(seed);
    std::normal_distribution<double> speed_noise(0.0, cfg.speed_noise_std);
    std::normal_distribution<double> steer_noise(0.0, cfg.steer_noise_std);

    std::vector<std::vector<std::pair<double, double>>> sequences;
    sequences.reserve(static_cast<size_t>(cfg.batch_size));
    for (int b = 0; b < cfg.batch_size; ++b) {
      std::vector<std::pair<double, double>> sequence;
      sequence.reserve(static_cast<size_t>(cfg.horizon_steps));
      for (int t = 0; t < cfg.horizon_steps; ++t) {
        double v = nominal_v[static_cast<size_t>(t)];
        double steer = nominal_steer[static_cast<size_t>(t)];
        if (b > 0) {
          v += speed_noise(rng);
          steer += steer_noise(rng);
        }
        sequence.emplace_back(clamp(v, min_speed, max_speed),
                              clamp(steer, -std::abs(cfg.max_steer),
                                    std::abs(cfg.max_steer)));
      }
      sequences.push_back(std::move(sequence));
    }

    std::vector<double> costs;
    costs.reserve(sequences.size());
    double min_cost = std::numeric_limits<double>::infinity();
    for (const auto &sequence : sequences) {
      const double cost = rollout_cost(initial, sequence, path_x, path_y, path_yaw, cfg);
      costs.push_back(cost);
      min_cost = std::min(min_cost, cost);
    }

    const double temperature = std::max(cfg.temperature, 1e-6);
    std::vector<double> weights(costs.size(), 0.0);
    double weight_sum = 0.0;
    for (size_t i = 0; i < costs.size(); ++i) {
      weights[i] = std::exp(-(costs[i] - min_cost) / temperature);
      weight_sum += weights[i];
    }
    weight_sum = std::max(weight_sum, 1e-9);

    std::vector<std::pair<double, double>> updated;
    updated.reserve(static_cast<size_t>(cfg.horizon_steps));
    for (int t = 0; t < cfg.horizon_steps; ++t) {
      double v = 0.0;
      double steer = 0.0;
      for (size_t i = 0; i < sequences.size(); ++i) {
        v += weights[i] * sequences[i][static_cast<size_t>(t)].first;
        steer += weights[i] * sequences[i][static_cast<size_t>(t)].second;
      }
      updated.emplace_back(clamp(v / weight_sum, min_speed, max_speed),
                           clamp(steer / weight_sum, -std::abs(cfg.max_steer),
                                 std::abs(cfg.max_steer)));
    }

    const double v0 = updated.front().first;
    const double steer0 = updated.front().second;
    const double yaw_rate =
        (std::abs(steer0) < 1e-5 || std::abs(v0) < 1e-5)
            ? 0.0
            : -v0 * std::tan(steer0) / std::max(cfg.wheelbase, 1e-6);

    PyObject *updated_list = PyList_New(static_cast<Py_ssize_t>(cfg.horizon_steps));
    if (!updated_list) {
      return nullptr;
    }
    for (int t = 0; t < cfg.horizon_steps; ++t) {
      PyObject *pair = Py_BuildValue("(dd)", updated[static_cast<size_t>(t)].first,
                                     updated[static_cast<size_t>(t)].second);
      if (!pair) {
        Py_DECREF(updated_list);
        return nullptr;
      }
      PyList_SET_ITEM(updated_list, static_cast<Py_ssize_t>(t), pair);
    }

    PyObject *result = Py_BuildValue("(dddO)", v0, steer0, yaw_rate, updated_list);
    Py_DECREF(updated_list);
    return result;
  } catch (const std::exception &exc) {
    PyErr_SetString(PyExc_RuntimeError, exc.what());
    return nullptr;
  }
}

PyMethodDef methods[] = {
    {"mppi_step", mppi_step, METH_VARARGS,
     "Run one native C++ MPPI update and return (v, steer, yaw_rate, updated_nominal)."},
    {nullptr, nullptr, 0, nullptr},
};

PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "_mppi_native",
    "Native C++ acceleration for controller-lab MPPI rollouts.",
    -1,
    methods,
};

}  // namespace

PyMODINIT_FUNC PyInit__mppi_native() { return PyModule_Create(&module); }
