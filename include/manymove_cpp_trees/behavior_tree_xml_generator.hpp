#ifndef MANYMOVE_CPP_TREES_BEHAVIOR_TREE_XML_GENERATOR_HPP
#define MANYMOVE_CPP_TREES_BEHAVIOR_TREE_XML_GENERATOR_HPP

#include <string>
#include <vector>
#include "manymove_cpp_trees/move.hpp"

namespace manymove_cpp_trees
{
    class BehaviorTreeXMLGenerator
    {
    public:
        // Constructor accepting the list of move sequences
        BehaviorTreeXMLGenerator(const std::vector<std::vector<Move>> &sequences);

        // Function to generate the XML string
        std::string generateXML() const;

    private:
        std::vector<std::vector<Move>> sequences_;
    };
} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_BEHAVIOR_TREE_XML_GENERATOR_HPP
