## We're facing issues because VILA and Remembr use different dependencies w.r.t. ROSA (They run in Python 3.10)
cd /app/remembr/
mkdir deps
cd deps
git clone https://github.com/NVlabs/VILA.git
cd /app/remembr/deps/VILA

python3.9 -m pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.8/flash_attn-2.5.8+cu122torch2.3cxx11abiFALSE-cp39-cp39-linux_x86_64.whl
python3.9 -m pip install -e .
python3.9 -m pip install -e ".[train]"
python3.9 -m pip install -e ".[eval]"

# Install HF's Transformers
#python -m pip install git+https://github.com/huggingface/transformers@v4.37.2
python3.9 -m pip install -U transformers==4.46.0
site_pkg_path=$(python -c 'import site; print(site.getsitepackages()[0])')
#cp -rv ./llava/train/transformers_replace/* $site_pkg_path/transformers/
#cp -rv ./llava/train/deepspeed_replace/* $site_pkg_path/deepspeed/

cd /app/remembr

## IF NEEDED
#python3.9 -m pip install --upgrade jpl-rosa
#python3.9 -m pip install --upgrade ollama
#python3.9 -m pip install --upgrade deepspeed

