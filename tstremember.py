from remembr.remembr.memory.memory import MemoryItem
from remembr.remembr.memory.milvus_memory import MilvusMemory
from remembr.remembr.memory.memory import MemoryItem

memory = MilvusMemory("test_collection", db_ip='127.0.0.1', db_port=19530)

memory.reset()

memory_item = MemoryItem(
    caption="I see a desk", 
    time=1.1, 
    position=[0.0, 0.0, 0.0], 
    theta=3.14
)

memory.insert(memory_item)
print("SUCCESS.")


