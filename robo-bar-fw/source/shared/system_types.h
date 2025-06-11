// system_types.h
#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

typedef enum {
    OP_IN_PROGRESS,
    OP_COMPLETED,  
    OP_ERROR,
    OP_TIMEOUT
} operation_status_t;

#endif /* SYSTEM_TYPES_H */