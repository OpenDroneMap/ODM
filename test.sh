if [ ! -z "$1" ]; then
	python3 -m unittest discover tests "test_$1.py"
else
	python3 -m unittest discover tests "test_*.py"
fi
