    #include <rosbag/bag.h>
    #include <rosbag/view.h>

    #include <boost/foreach.hpp>
    #define foreach BOOST_FOREACH

    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            ASSERT_EQ(s->data, std::string("foo"));

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            ASSERT_EQ(i->data, 42);
    }

    bag.close();
