import pytest

from openpilot.system.loggerd.parking_settings import parse_bool


@pytest.mark.parametrize("value", (b"1", b"true", b"TRUE", b" true "))
def test_parse_bool_true(value):
  assert parse_bool(value)


@pytest.mark.parametrize("value", (b"0", b"false", b"FALSE", b" false "))
def test_parse_bool_false(value):
  assert not parse_bool(value)


@pytest.mark.parametrize("value", (b"", b"yes", b"2", b"null"))
def test_parse_bool_invalid(value):
  with pytest.raises(ValueError):
    parse_bool(value)
