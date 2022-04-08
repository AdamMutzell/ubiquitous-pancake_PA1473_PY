def test_touchsensor():
    assert touchsensor.is_pressed() == False


def test_ultrasoundsensor():
    assert ultrasoundsensor.distance(2000) == False
    assert ultrasoundsensor.distance(1500) == False
    assert ultrasoundsensor.distance(1000) == False
    assert ultrasoundsensor.distance(500) == False
