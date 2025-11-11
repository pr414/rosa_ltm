cd /app/
apt-get update && apt-get install -y pciutils lshw
curl -fsSL https://ollama.com/install.sh | sh


#### FIX DEPENDENCIES:
# python3.9 -m pip install --upgrade jpl-rosa
# python3.9 -m pip install --upgrade ollama
# python3.9 -m pip install --upgrade deepspeed langchain-huggingface langchain-nvidia-ai-endpoints
