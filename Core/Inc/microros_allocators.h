/*
 * microros_allocators.h
 *
 *  Created on: Aug 24, 2024
 *      Author: Anton
 */

#ifndef INC_MICROROS_ALLOCATORS_H_
#define INC_MICROROS_ALLOCATORS_H_

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);



#endif /* INC_MICROROS_ALLOCATORS_H_ */
