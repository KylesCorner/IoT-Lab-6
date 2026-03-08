#pragma once
#include <Arduino.h>
#include "IActuator.h"

class L293dDcMotor : public IActuator {
public:
  enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Reverse
  };

  L293dDcMotor(
      uint8_t pinIn1,
      uint8_t pinIn2,
      uint8_t pinEn,
      const char* motorName = "L293D DC Motor")
      : _pinIn1(pinIn1),
        _pinIn2(pinIn2),
        _pinEn(pinEn),
        _name(motorName) {}

  const char* name() const override { return _name; }

  bool begin() override {
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
    pinMode(_pinEn, OUTPUT);

    stop();
    _healthy = true;
    _begun = true;
    return true;
  }

  void update() override {
    if (!_begun) return;

    applyOutputs();
  }

  bool healthy() const override {
    return _healthy;
  }

  // ---- Motor control API ----
  void stop() {
    _direction = Direction::Stop;
    _speed = 0;
    _dirty = true;
  }

  void forward(uint8_t speed) {
    _direction = Direction::Forward;
    _speed = speed;
    _dirty = true;
  }

  void reverse(uint8_t speed) {
    _direction = Direction::Reverse;
    _speed = speed;
    _dirty = true;
  }

  void setSpeed(uint8_t speed) {
    _speed = speed;
    _dirty = true;
  }

  void setDirection(Direction dir) {
    _direction = dir;
    if (_direction == Direction::Stop) {
      _speed = 0;
    }
    _dirty = true;
  }

  uint8_t speed() const { return _speed; }
  Direction direction() const { return _direction; }

private:
  uint8_t _pinIn1;
  uint8_t _pinIn2;
  uint8_t _pinEn;
  const char* _name;

  bool _healthy = false;
  bool _begun = false;
  bool _dirty = false;

  Direction _direction = Direction::Stop;
  uint8_t _speed = 0;

  void applyOutputs() {
    if (!_dirty) return;

    switch (_direction) {
      case Direction::Stop:
        digitalWrite(_pinIn1, LOW);
        digitalWrite(_pinIn2, LOW);
        analogWrite(_pinEn, 0);
        break;

      case Direction::Forward:
        digitalWrite(_pinIn1, HIGH);
        digitalWrite(_pinIn2, LOW);
        analogWrite(_pinEn, _speed);
        break;

      case Direction::Reverse:
        digitalWrite(_pinIn1, LOW);
        digitalWrite(_pinIn2, HIGH);
        analogWrite(_pinEn, _speed);
        break;
    }

    _dirty = false;
  }
};