#include <stdexcept>
#include "Utils/CircularBuffer.hpp"

namespace Leph
{
CircularBuffer::CircularBuffer(size_t size) : _buffer(size), _begin(0), _end(0)
{
}

size_t CircularBuffer::size() const
{
  return _buffer.size();
}

size_t CircularBuffer::count() const
{
  if (_begin == _end)
  {
    return 0;
  }
  else if (_end < _begin)
  {
    return size() - _begin + _end;
  }
  else
  {
    return _end - _begin;
  }
}

size_t CircularBuffer::dimension() const
{
  if (count() > 0)
  {
    return _buffer.front().size();
  }
  else
  {
    return 0;
  }
}

const VectorLabel& CircularBuffer::operator[](size_t index) const
{
  if (index + _begin >= size())
  {
    return _buffer[index - (size() - _begin)];
  }
  else
  {
    return _buffer[index + _begin];
  }
}

void CircularBuffer::add(const VectorLabel& vect)
{
  if (count() > 0 && vect.size() != dimension())
  {
    throw std::logic_error("CircularBuffer invalid dimension");
  }

  _buffer[_end] = vect;
  _end++;
  if (_end == size())
    _end = 0;
  if (_end == _begin)
    _begin++;
  if (_begin == size())
    _begin = 0;
}

VectorLabel CircularBuffer::mean() const
{
  if (count() == 0)
  {
    throw std::logic_error("CircularBuffer empty data");
  }

  VectorLabel sum = _buffer.front();
  sum.vect() = Vector::Zero(dimension());

  size_t index = _begin;
  size_t nb = 0;
  while (index != _end)
  {
    sum.addOp(_buffer[index]);
    index++;
    if (index == size())
      index = 0;
    nb++;
  }

  sum.mulOp(1.0 / nb);
  return sum;
}
VectorLabel CircularBuffer::variance() const
{
  if (count() == 0)
  {
    throw std::logic_error("CircularBuffer empty data");
  }

  Vector sum = Vector::Zero(dimension());
  VectorLabel sum2 = _buffer.front();
  sum2.vect() = Vector::Zero(dimension());

  size_t index = _begin;
  size_t nb = 0;
  while (index != _end)
  {
    sum += _buffer[index].vect();
    Vector tmp = (_buffer[index].vect().array()) * (_buffer[index].vect().array());
    sum2.vect() += tmp;
    index++;
    if (index == size())
      index = 0;
    nb++;
  }

  sum *= (1.0 / nb);
  sum2.mulOp(1.0 / nb);

  Vector mean2 = sum.array() * sum.array();
  sum2.vect() -= mean2;

  return sum2;
}

}  // namespace Leph
