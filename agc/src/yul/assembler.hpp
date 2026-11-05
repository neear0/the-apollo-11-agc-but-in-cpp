#pragma once

#include <array>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "agc/rope.hpp"

// an assembler for block ii agc source (Luminary, Comanche), producing the same rope as the original YUL assembler.
// it follows the rules of the Virtual AGC project's yaYUL, which was checked against the printed listings, so the
// output can be compared word for word with the ropes they transcribed
namespace yul
{
    // where a symbol or the location counter points: an erasable or fixed address, or a plain constant. sreg is the
    // 12-bit address the cpu sees, eb/fb/super the bank it's in, value a linear pseudo-address over all of memory:
    // erasable 0-3777, fixed-fixed 4000-7777, banked fixed 10000 + bank * 2000 (superbanks 40-43 at 110000 and up)
    struct address_t
    {
        bool invalid = false;  // not resolved yet
        bool constant = false; // a number rather than a location
        bool address = false;
        int sreg = 0;
        bool erasable = false;
        bool fixed = false;
        bool unbanked = false;
        bool banked = false;
        int eb = 0;
        int fb = 0;
        bool super = false;
        bool overflow = false; // the last increment ran off the end of its bank
        int value = 0;

        bool operator==(const address_t&) const = default;
    };

    address_t invalid_address();
    address_t pseudo_to_struct(int value); // invalid outside 0-117777
    address_t inc_pc(const address_t& pc, int increment);

    // an octal number, or a decimal one ending in d or containing 8 or 9, with an optional sign
    std::optional<int> parse_number(std::string_view text);

    // big: keep it off the stack where it can be, the tables by bank are on the heap already
    struct assembly_t
    {
        agc::rope_t rope{};
        std::vector<std::array<bool, agc::bank_size>> used = std::vector<std::array<bool, agc::bank_size>>(agc::fixed_bank_count); // words the source (or a bugger word) filled
        std::vector<std::array<int, agc::bank_size>> origin = std::vector<std::array<int, agc::bank_size>>(agc::fixed_bank_count); // index into sources of the line a word came from, -1 for none
        std::vector<std::string> sources;                                                                                           // "FILE.agc:line"
        std::unordered_map<std::string, address_t> symbols;
        std::vector<std::string> warnings;
    };

    // assembles main_file and everything it includes ($FILE.agc lines, relative to it). the errors on failure
    std::expected<assembly_t, std::vector<std::string>> assemble(const std::filesystem::path& main_file);

    // ------------------------------------------------------------------------------------------------------------
    // the assembler itself, assemble() runs it

    struct source_line_t
    {
        std::string file;
        int line_number = 0;
        std::string text;                // tabs expanded, comment removed
        std::vector<std::string> fields; // up to 6, split on whitespace
        int first_column = 0;            // where fields[0] starts
    };

    struct ebank_t
    {
        bool oneshot_pending = false;
        address_t current;
        address_t last;
    };

    struct sbank_t
    {
        bool oneshot_pending = false;
        unsigned current = 0; // 0 not established yet, 3 or 4 once code is in banks 30-37 or 40-43
        unsigned last = 0;
    };

    struct line_input_t
    {
        address_t pc;
        std::string label;
        std::string false_label;
        std::string op;
        std::string operand;
        std::string mod1;
        std::string mod2;
        std::string extra;
        std::string alias;
        int extend = 0; // 1 after EXTEND, 2 after a TC 6 written out
        bool index_valid = false;
        ebank_t ebank;
        sbank_t sbank;
    };

    inline constexpr int illegal_word = static_cast<int>(0x80000000u);

    struct line_output_t
    {
        address_t pc;
        std::array<int, 2> words{ illegal_word, illegal_word };
        int num_words = 0;
        std::string error;
        bool warning = false;
        bool fatal = false;
        address_t label_value;
        bool label_value_valid = false;
        int extend = 0;
        bool index_valid = false;
        ebank_t ebank;
        sbank_t sbank;
        bool equals = false; // the line defined its label itself (= or EQUALS)
    };

    enum class op_type { basic, pseudo, interpreter, downlink };

    enum class parser_kind
    {
        none, general, tcf, index, edrupt, st_call, st_odl, st_ore, st_ovl,
        equals, eq_minus, erase, setloc, bank, block, ebank_equals, sbank_equals,
        cadr, two_cadr, ecadr, genadr, bbcon, bbcon_star, oct, two_oct, dec, two_dec, vn
    };

    struct parser_match_t
    {
        std::string_view name;
        op_type type = op_type::basic;
        parser_kind kind = parser_kind::none;
        int opcode = 0;
        int flags = 0;
        std::string_view alias_op;      // with parser_kind::none: the operator this one stands for, none and empty: ignored
        std::string_view alias_operand;
        int adder = 0;                  // added to the first word and then xor-ed with xmask, for the downlink forms
        int xmask = 0;
        int adder2 = 0;
        int xmask2 = 0;
        bool pinch_hit = false;         // can stand in for an interpretive operand
    };

    struct interpreter_match_t
    {
        std::string_view name;
        int code = 0;
        int num_operands = 0;
        int nnnn0000 = 0;                // switch (low bits 1) and shift (low bits 2) operand encodings
        std::array<int, 2> arg_types{}; // 1: the operand word is address + 1, complemented when indexed by x2
    };

    class c_assembler
    {
    public:
        explicit c_assembler(std::filesystem::path main_file);

        std::expected<assembly_t, std::vector<std::string>> run();

    private:
        bool load(const std::filesystem::path& file, int depth);
        void collect_symbols();
        void pass(bool write);
        void add_bugger_words();

        address_t* find_symbol(const std::string& name);
        void edit_symbol(const std::string& name, const address_t& value);
        bool fetch_symbol_plus_offset(const address_t& pc, const std::string& operand, const std::string& mod1, address_t& value);

        void start_bank_counts();
        void update_bank_counts(const address_t& pc);

        int parse_comma(line_input_t& in);
        void parse(const parser_match_t& match, line_input_t& in, line_output_t& out);
        void parse_general(line_input_t& in, line_output_t& out, int opcode, int flags);
        void parse_st(line_input_t& in, line_output_t& out, int opcode, int flags);
        void parse_interpretive_operand(line_input_t& in, line_output_t& out);
        void parse_index(line_input_t& in, line_output_t& out);
        void parse_equals(line_input_t& in, line_output_t& out);
        void parse_eq_minus(line_input_t& in, line_output_t& out);
        void parse_erase(line_input_t& in, line_output_t& out);
        void parse_setloc(line_input_t& in, line_output_t& out);
        void parse_bank(line_input_t& in, line_output_t& out);
        void parse_block(line_input_t& in, line_output_t& out);
        void parse_ebank_equals(line_input_t& in, line_output_t& out);
        void parse_sbank_equals(line_input_t& in, line_output_t& out);
        void parse_cadr(line_input_t& in, line_output_t& out);
        void parse_2cadr(line_input_t& in, line_output_t& out);
        void parse_ecadr(line_input_t& in, line_output_t& out);
        void parse_genadr(line_input_t& in, line_output_t& out);
        void parse_bbcon(line_input_t& in, line_output_t& out, bool star);
        void parse_oct(line_input_t& in, line_output_t& out);
        void parse_2oct(line_input_t& in, line_output_t& out);
        void parse_dec(line_input_t& in, line_output_t& out);
        void parse_2dec(line_input_t& in, line_output_t& out);
        void parse_vn(line_input_t& in, line_output_t& out);

        std::filesystem::path m_main_file;
        std::vector<source_line_t> m_lines;
        std::vector<std::string> m_errors;

        std::unordered_map<std::string, address_t> m_symbols;
        int m_symbols_reassigned = 0;

        assembly_t m_assembly;
        std::array<int, agc::fixed_bank_count> m_used_in_bank{};
        std::array<int, agc::fixed_bank_count> m_prior_used_in_bank{};

        // interpretive state, carried from an opcode line to its operand lines
        int m_num_interpretive_operands = 0;
        int m_raw_num_interpretive_operands = 0;
        std::array<int, 4> m_switch_increment{};
        std::array<int, 4> m_nnnn_fields{};
        int m_stadr_invert = 0;

        // per line
        int m_opcode_offset = 0;
        int m_arg_type = 0;
        const source_line_t* m_line = nullptr;
    };
}
