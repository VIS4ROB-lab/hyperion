<html>

<head>
  <meta name="color-scheme" content="light dark">
</head>

<body>
<div align="center">
    <a href="https://github.com/VIS4ROB-lab/hyperion">
        <img width="300" src="resources/images/hyperion_logo.png" alt="">
    </a>
    <br/><br/>
    A high-performance Continuous-Time Gaussian Belief Propagation (CT-GBP) framework with fully
    automated symbolic factor generation and seamless Ceres interoperability targeting
    distributed SLAM operations!
    <br/><br/>
    <a href="https://github.com/VIS4ROB-lab/hyperion/issues">
        Report Issues or Request Features
    </a>
</div>

<br/><br/><br/>

<h1> About </h1>

Hyperion is a novel, modular, distributed, high-performance optimization framework targeting both discrete- and
continuous-time SLAM (Simultaneous Localization and Mapping) applications. It stands out by offering the
first open-source C++ implementation of a Gaussian-Belief-Propagation-based Non-Linear Least Squares solver, which,
in turn, offers native support for decentralized, stochastic inference on factor graphs. In addition, Hyperion also
extends [SymForce](https://github.com/symforce-org/symforce) to automate the generation of high-performance
implementations for spline-related residuals from symbolic, high-level expressions. This results in the fastest,
[Ceres](http://ceres-solver.org)-interoperable B- and Z-Spline implementations, achieving speedups of up to 110x over
previous state-of-the-art methods.
<br/><br/>

<h3> Release </h3>

Hyperion will officially be released prior to
the [European Conference on Computer Vision 2024 (ECCV 2024)](https://eccv2024.ecva.net),
scheduled until September 29, 2024. Thus, stay tuned for updates and prepare to accelerate and streamline your own
continuous-time SLAM framework or spline-related research with Hyperion.
<br/><br/>

<h3> Citation </h3>

Hyperion has been accepted for publication in
the [European Conference on Computer Vision 2024 (ECCV 2024)](https://eccv2024.ecva.net). Until the final version of
record becomes available, please cite its archived version as follows:

```
@misc{Hug:etal:arXiv2024,
      title={{Hyperion -- A fast, versatile symbolic Gaussian Belief Propagation framework for Continuous-Time SLAM}}, 
      author={David Hug and Ignacio Alzugaray and Margarita Chli},
      year={2024},
      eprint={2407.07074},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2407.07074}, 
}
```
<br/><br/>

<!-- <h1> Setup </h1> -->

<div align="center">
    <h3> Contact </h3>
    Admin - <a href="mailto:dhug@ethz.ch">David Hug</a>, Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland
</div>

<div align="center">
    <h3> License </h3>
    Hyperion is distributed under the <a href="LICENSE">BSD-3-Clause License</a>
    <br/><br/><br/>
    <a href="https://asl.ethz.ch/v4rl">
        <source srcset="resources/images/v4rl_logo_light.png" media="(prefers-color-scheme: dark)">
    	<img width="150" src="resources/images/v4rl_logo_dark.png">
    </a>
</div>
</body>

</html>