#ifndef POLY_H
#define POLY_H
#include <stdint.h>
#include <string.h>

#if !defined RS_DEBUG && !defined __CC_ARM && !defined RS_NO_ASSERT
#include <assert.h>
#else
#define assert(dummy)
#endif

namespace RS {

struct Poly {
    Poly()
        : length(0), _memory(NULL) {}

    Poly(uint8_t id, uint16_t offset, uint8_t size) \
        : length(0), _id(id), _size(size), _offset(offset), _memory(NULL) {}

    inline bool Append(uint8_t num) {
        assert(length+1 < _size);
        ptr()[length++] = num;
        return true;
    }

    inline void Init(uint8_t id, uint16_t offset, uint8_t size, uint8_t** memory_ptr) {
        this->_id     = id;
        this->_offset = offset;
        this->_size   = size;
        this->length  = 0;
        this->_memory = memory_ptr;
    }

    inline void Reset() {
        memset((void*)ptr(), 0, this->_size);
    }

    inline void Set(const uint8_t* src, uint8_t len, uint8_t offset = 0) {
        assert(src && len <= this->_size-offset);
        memcpy(ptr()+offset, src, len * sizeof(uint8_t));
        length = len + offset;
    }

    #define poly_max(a, b) ((a > b) ? (a) : (b))

    inline void Copy(const Poly* src) {
        length = poly_max(length, src->length);
        Set(src->ptr(), length);
    }

    inline uint8_t& at(uint8_t i) const {
        assert(i < _size);
        return ptr()[i];
    }

    inline uint8_t id() const {
        return _id;
    }

    inline uint8_t size() const {
        return _size;
    }

    inline uint8_t* ptr() const {
        assert(_memory && *_memory);
        return (*_memory) + _offset;
    }

    uint8_t length;

protected:

    uint8_t   _id;
    uint8_t   _size;
    uint16_t  _offset;
    uint8_t** _memory;
};

}

#endif
