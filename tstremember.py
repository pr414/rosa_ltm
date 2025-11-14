# For proper running, run:
# pip install "transformers==4.43.3" "peft==0.11.1" "sentence-transformers==2.7.0"

from remembr.memory.milvus_memory import MilvusMemory
from remembr.memory.memory import MemoryItem
from remembr.agents.remembr_agent import ReMEmbRAgent

# Run MILVUS before instanciating MilvusMemory
memory = MilvusMemory("test_collection", db_ip='127.0.0.1', db_port=19530)

memory.reset()

memory_item = MemoryItem(
    caption="I see a desk", 
    time=1.1, 
    position=[0.0, 0.0, 0.0], 
    theta=3.14
)
memory.insert(memory_item)

memory_item = MemoryItem(
    caption="I see a big fridge", 
    time=1.1, 
    position=[4.0, 2.0, 0.0], 
    theta=3.14
)
memory.insert(memory_item)

memory_item = MemoryItem(
    caption="I see a nice window looking outside", 
    time=1.1, 
    position=[9.0, 2.0, 0.0], 
    theta=3.14
)
memory.insert(memory_item)

print("MEMORY BUILD SUCCESS.")

agent = ReMEmbRAgent(llm_type='gpt-4')
agent.set_memory(memory)
response = agent.query("Take me to a fridge")
print(response.position)

# uncomment if you want to see the text reason for the position data
print(response.text) 