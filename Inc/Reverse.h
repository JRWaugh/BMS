/*
 * Reverse.h
 *
 *  Created on: Mar 16, 2020
 *      Author: Joshua
 */

#ifndef REVERSE_H_
#define REVERSE_H_

// Taken from  https://stackoverflow.com/a/42221591

template<typename Range>
class Reverse {
public:
    Reverse(Range const& range) : range { range } {}

    auto begin() const {
    	return std::make_reverse_iterator(std::end(range));
    }
    auto end() const {
    	return std::make_reverse_iterator(std::begin(range));
    }

private:
    Range const& range;
};

template<typename Range>
auto reverse(Range const& range) {
	return Reverse<Range>{ range };
}



#endif /* REVERSE_H_ */
