# execute specific test with
python -m unittest mypkg.tests.test_module.TestClass.test_method

# execute all test in file with
python3 -m unittest tests/this_test_file.py

# execute all test with, make sure all test files end with _test.py
python -m unittest discover -s ./tests/ -p '*_test.py'

