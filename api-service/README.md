# Installation and Execution
1. Install uv: https://docs.astral.sh/uv/getting-started/installation
2. Install requirements
```sh
# make sure you are in the api-service directory
uv init . --bare
uv add "fastapi[standard]"
```
3. To start the app:
```sh
uv run fastapi dev api-service.py
```