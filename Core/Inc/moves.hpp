/*
 * moves.hpp
 *
 *  Created on: Apr 12, 2022
 *      Author: george
 */

#ifndef INC_MOVES_HPP_
#define INC_MOVES_HPP_

#include <queue>
#include <memory>
#include <functional>

template<typename Output>
struct move
{
	enum status
	{
		NO_UPDATE,
		UPDATED,
		FINISHED
	};

	virtual ~move() {}

	/* called periodically, this is where the move's state should update,
	if this returns UPDATED then the get() will be called to update the output,
	if FINISHED the move gets destroyed and the manager begins processing
	next move in the queue */
	virtual status update() = 0;
	virtual Output get() = 0;
};


template<typename Output>
class move_manager
{
	using status = move<Output>::status;
	
	std::queue<std::unique_ptr<move<Output>>> _moves;
	std::function<void(Output)> _callback;
	bool first;

public:

	move_manager(std::function<void(Output)> callback) :
		_callback(std::move(callback)), first(true) {}

	void update()
	{
		if (!_moves.empty())
		{
			auto s = _moves.front()->update();
			
			if (s == status::UPDATED || first)
			{
				_callback(std::move(_moves.front()->get()));
				first = false;
			}
			
			else if (s == status::FINISHED)
			{
				_moves.pop();
				if (_moves.empty())
				{
					_callback(Output());
					first = true;
				}
				else
					_callback(std::move(_moves.front()->get()));
			}
		}
		else
			first = true;
	}

	template<class Move, typename... Args>
	void next_move(Args... args)
	{
		auto m = new Move(std::forward<Args>(args)...);
		_moves.push(std::unique_ptr<move<Output>>(m));
	}

	void cancel_all()
	{
		while (!_moves.empty())
			_moves.pop();

		_callback(Output());
	}

	bool is_moving()
	{
		return !_moves.empty();
	}
};


#endif /* INC_MOVES_HPP_ */
