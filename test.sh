if [ ! -z "$1" ]; then
	python -m unittest discover tests "test_$1.py"
else
	python -m unittest discover tests "test_*.py"
fi
