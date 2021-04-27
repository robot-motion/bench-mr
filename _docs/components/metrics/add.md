---
layout: default
title:  "Add a Custom Metric"
date:   2021-01-04 13:20:59 +0100
parent: Metrics
grand_parent: Components
nav_order: 3
---

# Add a Custom Metric

To add a new metric to Bench-MR, follow these steps:

## Inherit from `TMetric`

Implement an interface to the C++ implementation of your metric that inherits from `TMetric` and follows the curiously recurring template pattern (CRTP):

```cpp
class MyMetric : public TMetric<MyMetric> {
 public:
  static double evaluateMetric(const ompl::geometric::PathGeometric& trajectory, double dt) {
    // ...
  }

  static double evaluateMetric(const ompl::control::PathControl& trajectory, double dt) {
    // ...
  }
};
```

As shown, the two types of input paths (geometric and control-based) follow the OMPL API.

## Add Property to `PathStatistics`

Register the new metric in `base/PathStatistics.hpp` in the `PathStatistics` property group:

```cpp
Property<double> my_metric{
      std::numeric_limits<double>::quiet_NaN(), "my_metric", this};
```

## Evaluate Metric in `PathEvaluation::evaluate`

Include the evaluation of the new metric in `PathEvaluation::evaluate` (in `utils/PathEvaluation.hpp`):

```cpp
stats.my_metric = MyMetric::evaluate(solution);
```

## Update Front-end Definitions

Go to `python/definitions.py` to optionally register a user-friendly title in the `stat_names` dictionary that is displayed in the plots inplace of the metric name.

```py
stat_names = {
    # ...
    'my_metric': 'My Metric'
}
```
