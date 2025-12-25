
import pytest

from math_utils import add, divide

def test_add():
    assert add(2, 3) == 5

def test_divide_by_zero():
    with pytest.raises(ValueError):
        divide(10, 0)

@pytest.fixture
def sample_list():
    return [1, 2, 3]

def test_list_length(sample_list):
    assert len(sample_list) == 3

@pytest.mark.parametrize(
    "a, b, expected",
    [
        (1, 2, 3),
        (0, 0, 0),
        (-1, 1, 0),
    ]
)
def test_add_many_cases(a, b, expected):
    assert a + b == expected
