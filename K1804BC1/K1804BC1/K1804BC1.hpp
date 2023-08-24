#pragma once

#include <model.hpp>
#include <array>

class K1804BC1 final: public vsm::model
{
public:
	static constexpr DWORD MODEL_KEY = 0x00000002;

private:
	static constexpr uint8_t WORD_SIZE = 4;
	static constexpr uint8_t RON_SIZE = 16;

	using sel_pins = std::array<vsm::pin, 9>;
	using ws_pins = std::array<vsm::pin, WORD_SIZE>;
	using value_type = uint8_t;

private:
	std::array<value_type, RON_SIZE> RON;
	uint8_t PQ;
	
	sel_pins _pins_I;
	ws_pins _pins_A;
	ws_pins _pins_B;
	ws_pins _pins_D;
	ws_pins _pins_Y;
	ws_pins _pins_F;

	vsm::pin OE;
	vsm::pin CLK;
	vsm::pin C0;

public:
	K1804BC1();

	VOID setup(IINSTANCE *inst, IDSIMCKT *dsim);
	VOID simulate(ABSTIME time, DSIMMODES mode);

	~K1804BC1() = default;

private:
	std::pair<value_type, value_type> get_operands();
	int get_result(value_type r, value_type s, bool &ovr, bool &c4, bool &f3, bool &z);
	void write_result(ABSTIME time, value_type res, bool ovr, bool c4, bool f3, bool z);
};
