# To launch Remembr correctly:

Launch MILVIUS docker file with:
    cd milviusconfig
    bash milvius_patch start
(If we need to delete image, run bash milvius_patch stop, bash milvius_patch delete, docker image rm milvius...)

Then launch the main docker with
    ./demo.sh

Once inside, set remembr venv and launch the test:
    source /opt/venv/remembr/bin/activate
    python3 tstremember.py
To work, set beforehand the OPENAI API KEY inside .env
