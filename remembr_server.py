#!/usr/bin/env python3
from flask import Flask, request, jsonify
from remembr.agents.remembr_agent import ReMEmbRAgent
from remembr.memory.milvus_memory import MilvusMemory
from remembr.memory.memory import MemoryItem

app = Flask(__name__)

# Initialize memory + agent
memory = MilvusMemory("test_collection", db_ip="127.0.0.1", db_port=19530)
memory.reset()

memory_item = MemoryItem(
    caption="I see a desk", 
    time=1.1, 
    position=[-6.0, 2.0, 0.0], 
    theta=3.14
)
memory.insert(memory_item)

memory_item = MemoryItem(
    caption="I see a big fridge", 
    time=1.1, 
    position=[7.64, -1.07, 0.0], 
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

ltm_agent = ReMEmbRAgent(llm_type="gpt-4")
ltm_agent.set_memory(memory)


print("Remembr server ready!")

@app.route("/query", methods=["POST"])
def query():
    data = request.get_json()
    query_text = data.get("query", "")
    if not query_text:
        return jsonify({"error": "No query provided"}), 400
    try:
        response = ltm_agent.query(query_text)
        pos = getattr(response, "position", None)
        return jsonify({
            "text": response.text,
            "position": pos if pos else None
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
