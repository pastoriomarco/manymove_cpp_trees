#ifndef MANYMOVE_CPP_TREES_BEHAVIOR_TREE_XML_GENERATOR_HPP
#define MANYMOVE_CPP_TREES_BEHAVIOR_TREE_XML_GENERATOR_HPP

#include <string>
#include <vector>
#include "manymove_cpp_trees/move.hpp"

namespace manymove_cpp_trees
{
    /**
     * @class BehaviorTreeXMLGenerator
     * @brief A utility class to generate a Behavior Tree XML description based on a sequence of moves.
     *
     * This class takes one or more sequences of Move objects and creates a single
     * Behavior Tree XML string to orchestrate the planning and execution of
     * the moves in sequence.
     */
    class BehaviorTreeXMLGenerator
    {
    public:
        /**
         * @brief Constructor that accepts the list of move sequences.
         * @param sequences A vector of sequences, where each sequence is a vector of Move objects.
         */
        BehaviorTreeXMLGenerator(const std::vector<std::vector<Move>> &sequences);

        /**
         * @brief Generate the Behavior Tree XML string.
         * @return A string containing the XML definition of the Behavior Tree.
         */
        std::string generateXML() const;

    private:
        std::vector<std::vector<Move>> sequences_; ///< Internal storage of move sequences.
    };
} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_BEHAVIOR_TREE_XML_GENERATOR_HPP
