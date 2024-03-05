# (Meshed) UWB Anchor Initialization Framework



| Trajectory | Measurements |
|:----------|:-------------|
| ![](./examples/cmd/data/Trajectory-Anchor-Pos-Estimates.png) | ![](./examples/cmd/data/DS-TWR-outliers-0.15-noise-d0.1-noise-T0.1-measurements.png) 


## Credit

This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), 
University of Klagenfurt, Klagenfurt, Austria.

## License

This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-
License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in
patents is granted.

### Usage for academic purposes

If you use this software in an academic research setting, please cite the
corresponding [academic paper] and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{Delama2023,
   author       = {Delama, Giulio and Shamsfakhr, Farhad and Weiss, Stephan and Fontanelli, Daniele and Fornasier, Alessandro},
   booktitle    = {2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
   title        = {UVIO: An UWB-Aided Visual-Inertial Odometry Framework with Bias-Compensated Anchors Initialization},
   year         = {2023},
  organization  = {IEEE}
}
```

```latex
@inproceedings{Jung2024,
   author       = {Jung, Roland and Santoro, Luca and Brunelli, Davide and Fontanelli, Daniele and Weiss, Stephan},
   booktitle    = {tbd},
   title        = {Modular Meshed Ultra-Wideband Aided Inertial Navigation with Robust Anchor Calibration},
   year         = {2024},
  organization  = {tbd}
}
```

### Build

1. The C++ stand-alone library can also be builded and installed with:
```[bash]
cd uwb_init_lib
mkdir build && cd build
cmake ../
cmake --build .
sudo cmake --install .
```

## Usage

The objective is to initialize a set of unknown UWB anchors. To understand the motivation and the detailed initialization procedure
please refer to the [academic paper].


## Reporting Issues

In case of issues, feature requests, or other questions please open a [New Issue](https://gitlab.aau.at/aau-cns/ros_pkgs/uwb_init_cpp/issues/new?issue) or contact the authors via email.

## Authors

* Giulio Delama ([email](mailto:giulio.delama@aau.at?subject=[UWB%20Init]))
* Alessandro Fornasier ([email](mailto:alessandro.fornasier@ieee.org?subject=[UWB%20Init]))
* Martin Scheiber ([email](mailto:martin.scheiber@ieee.org?subject=[UWB%20Init]))
* Roland Jung ([email](mailto:roland.jung@ieee.org?subject=[UWB%20Init]))

<!-- LINKS: -->
[academic paper]: https://arxiv.org/abs/2308.00513
