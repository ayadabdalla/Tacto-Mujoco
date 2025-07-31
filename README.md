## Installation Instructions

Follow the steps below to set up the environment and install the required dependencies:

1. **Clone the repository**:
    ```bash
    git clone https://github.com/ayadabdalla/Tacto-Mujoco.git
    cd Tacto-Mujoco
    ```

2. **Create a virtual environment**:
    ```bash
    python3 -m venv .tacto_mujoco
    source .tacto_mujoco/bin/activate
    ```

3. **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    pip install --no-deps git+https://github.com/ayadabdalla/tacto.git@18252b8cc86f2b12efc2a5a3ff3cac4e08308546
    pip install --no-deps urdfpy==0.0.22
    pip install -e .
    ```

## Running the Demo

To run the demo script and interact with the simulation:

1. **Execute the demo script**:
    ```bash
    python src/tacto_mujoco/scripts/tacto_examples/demo_mujoco_digit.py
    ```

2. **Controls**:
    - **Move the object**: Use `W`, `A`, `S`, `D`, `R`, `F` keys.
    - **Move the sensor**: Use `Up`, `Down`, `Left`, `Right`, `Page Up`, `Page Down` keys.

    ## Exploring the Sensor in MuJoCo

    To explore the sensor in the MuJoCo environment, run the following script:

    ```bash
    python src/tacto_mujoco/scripts/touch_playground_mujoco_launcher.py
    ```