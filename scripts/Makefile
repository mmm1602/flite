PY=python3

.PHONY: setup check solve demo

setup:
	python3 -m venv .venv && . .venv/bin/activate && pip install -r solver/requirements.txt

check:
	. .venv/bin/activate && $(PY) solver/check.py

solve:
	. .venv/bin/activate && $(PY) solver/solve_routes.py \
	  --data-dir ./data \
	  --out ./app/public/routes.json \
	  --units feet --max-cap 37725 --vrp --vrp-vehicles 32 --seed 0 \
	  --polygon ./data/polygon_lon_lat.wkt

demo:
	cd app && npm ci && npm run build && npx serve dist