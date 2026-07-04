#!/bin/bash

echo "========================================="
echo "Setting up Virtual Environment to fix NumPy..."
echo "========================================="

# 1. Create a virtual environment named 'venv' in the current directory
python3 -m venv venv

# 2. Activate the virtual environment
source venv/bin/activate

# 3. Upgrade pip
pip install --upgrade pip

# 4. Install compatible versions of numpy and matplotlib
echo "Installing compatible packages..."
pip install "numpy<2" matplotlib

echo "========================================="
echo "Running the simulation..."
echo "========================================="

# 5. Run the simulation
python3 simulate_dynamic.py

echo "========================================="
echo "Done! The simulation GIF should be saved."
echo "Note: Next time, you can just run: source venv/bin/activate && python3 simulate_dynamic.py"
echo "========================================="
