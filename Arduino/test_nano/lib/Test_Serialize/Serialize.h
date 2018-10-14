#ifndef SERIALIZE_H
#define SERIALIZE_H

template <typename T>
    union Serialize {
        T data;
        uint8_t bytes[sizeof(T)];

        const uint8_t *begin() { return bytes; }
        const uint8_t *end() { return bytes + sizeof(T); }
};

#endif