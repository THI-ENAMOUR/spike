if ! command -v pytest &> /dev/null
then
	sudo apt install -y python-pytest
fi

pytest src/enamour/
