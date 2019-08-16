import lazyros.time_buffer
import pytest


def test_time_buffer():
    tb = lazyros.time_buffer.TimeBuffer(capacity=3)
    tb.append(1.0, "Hallo")
    assert len(tb) == 1
    assert tb.get_latest(1.0) == "Hallo"
    assert tb.get_closest(1.0) == "Hallo"
    tb.append(3.0, "ABC")
    assert tb.get_closest(1.5) == "Hallo"
    assert tb.get_closest(2.6) == "ABC"
    assert tb.get_latest(2.99) == "Hallo"
    assert tb.get_latest(3.0) == "ABC"
    assert tb.get_latest(2.0) == "Hallo"
    tb.append(2.0, "XYZ")
    assert tb.get_closest(2.5) == "XYZ"
    assert tb.get_latest(3.0) == "ABC"
    assert tb.get_latest(2.0) == "XYZ"
    tb.append(4.0, "HALLO")
    assert tb.get_closest(1.0) == "XYZ"
    assert len(tb) == 3

def test_dict_in_time_buffer():
    tb = lazyros.time_buffer.TimeBuffer(capacity=3)
    tb.append(1.0, {1:"Hallo"})
    tb.append(2.0, {2:"ABC"})
    assert tb.get_closest(1.4) == {1: "Hallo"}
    assert tb.get_closest(1.55) == {2: "ABC"}


def test_static_type():
    tb = lazyros.time_buffer.TimeBuffer(capacity=3)
    tb.append(1.0, "test")
    with pytest.raises(RuntimeError):
        tb.append(1, "dada")
