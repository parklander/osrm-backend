@routing  @guidance @turn-angles
Feature: Simple Turns

    Background:
        Given the profile "car"
        Given a grid size of 1 meters

    Scenario: Offset Turn
        Given the node map
            | a |   |   |   |   |   |   |   |   | b |   |   |   |   |   |   |   |   |   | c |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   | d |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
            |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   | e |   |

        And the ways
            | nodes  | highway | name | lanes |
            | abc    | primary | road | 4     |
            | bde    | primary | turn | 2     |

       When I route I should get
            | waypoints | route          | turns                           |
            | a,c       | road,road      | depart,arrive                   |
            | a,e       | road,turn,turn | depart,turn slight right,arrive |
            | e,a       | turn,road,road | depart,turn slight left,arrive  |
            | e,c       | turn,road,road | depart,turn sharp right,arrive  |
