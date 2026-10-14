#include "utils/output_bit_stream.h"

#include <algorithm>

OutputBitStream::OutputBitStream(uint32_t buffer_size) {
  data_ = Array<uint32_t>(std::max<uint32_t>(buffer_size / 4 + 1, 1));
  std::fill(data_.begin(), data_.end(), 0);
  buffer_ = 0;
  cursor_ = 0;
  bit_in_buffer_ = 0;
}

void OutputBitStream::EnsureCapacity(uint32_t required_words) {
  if (required_words <= static_cast<uint32_t>(data_.length())) return;

  uint32_t new_capacity = std::max(required_words, std::max<uint32_t>(1, static_cast<uint32_t>(data_.length()) * 2));
  Array<uint32_t> new_data(new_capacity);
  std::fill(new_data.begin(), new_data.end(), 0);
  std::copy(data_.begin(), data_.begin() + cursor_, new_data.begin());
  data_ = new_data;
}

uint32_t OutputBitStream::Write(uint64_t content, uint32_t len) {
  content <<= (64 - len);
  buffer_ |= (content >> bit_in_buffer_);
  bit_in_buffer_ += len;
  if (bit_in_buffer_ >= 32) {
    EnsureCapacity(cursor_ + 1);
    data_[cursor_++] = (buffer_ >> 32);
    buffer_ <<= 32;
    bit_in_buffer_ -= 32;
  }
  return len;
}

uint32_t OutputBitStream::WriteLong(uint64_t content, uint64_t len) {
  if (len == 0) return 0;
  if (len > 32) {
    Write(content >> (len - 32), 32);
    Write(content, len - 32);
    return len;
  }
  return Write(content, len);
}

uint32_t OutputBitStream::WriteInt(uint32_t content, uint32_t len) {
  return Write(static_cast<uint64_t>(content), len);
}

uint32_t OutputBitStream::WriteBit(bool bit) {
  return Write(static_cast<uint64_t>(bit), 1);
}

Array<uint8_t> OutputBitStream::GetBuffer(uint32_t len) {
  Array<uint8_t> ret(len);
  uint32_t word_count = (len + sizeof(uint32_t) - 1) / sizeof(uint32_t);
  for (uint32_t i = 0; i < word_count; ++i) {
    uint32_t block = htobe32(data_[i]);
    uint32_t copy_len = std::min<uint32_t>(sizeof(uint32_t), len - i * sizeof(uint32_t));
    __builtin_memcpy(ret.begin() + i * sizeof(uint32_t), &block, copy_len);
  }
  return ret;
}

void OutputBitStream::Flush() {
  if (bit_in_buffer_) {
    EnsureCapacity(cursor_ + 1);
    data_[cursor_++] = buffer_ >> 32;
    buffer_ = 0;
    bit_in_buffer_ = 0;
  }
}

void OutputBitStream::Refresh() {
  cursor_ = 0;
  bit_in_buffer_ = 0;
  buffer_ = 0;
}
