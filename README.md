<h1 align="center">
Data-fused Model Predictive Control with Guarantees: Application to Flying Humanoid Robots
</h1>

<div align="center">
Davide Gorbani, Mohamed Elobaid, Giuseppe L'Erario, Hosameldin Awadalla Omer Mohamed, Daniele Pucci
<br>
<b>Co-first authors: Davide Gorbani and Mohamed Elobaid</b>
</div>
<br>

<div align="center">
    <a href="https://arxiv.org/abs/2509.10353"><b>📚 arXiv</b></a> &nbsp;&nbsp;&nbsp;
    <a href="#Usage"><b>🔧 Usage</b></a>
</div>
<br>


https://github.com/user-attachments/assets/f9a0b127-e5e7-4258-9e86-18793648615e


## Dependency

This project uses [pixi](https://prefix.dev/docs/pixi) for environment and dependency management. You can install pixi by following [this installation guide](https://pixi.sh/latest/installation/).

## Usage

> [!IMPORTANT]
> The installation has been tested only on Ubuntu and macOs systems.

To run the simulation, execute in the terminal the command:

```bash
pixi run run-simulation
```

This will install the project and execute the Python script to run the Mujoco simulation of the iRonCub robot.

> [!NOTE]
> The simulation might be slow since the jet dynamics is simulated using a neural network, and the inference of the neural network slows down the simulation.
> 
> At the start of the simulation, before the robot takes off, its arms shake due to the tuning of the joints position regularization weight. This tuning improves performance and responsiveness during flight, but we are working on better tuning and gain scheduling for both phases to eliminate the shaking.

If you want to run the toy problem, run:
```bash
pixi run simple-example
```

## Acknowledgement
This work utilized the [blend_my_bot](https://github.com/ami-iit/blend-my-bot) library for render generation.

## Maintainers

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/davidegorbani">
        <img src="https://github.com/davidegorbani.png" width="80" alt="Davide Gorbani"><br>
        👨‍💻 Davide Gorbani
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/mebbaid">
        <img src="https://github.com/mebbaid.png" width="80" alt="Mohamed Elobaid"><br>
        👨‍💻 Mohamed Elobaid
      </a>
    </td>
  </tr>
</table>
