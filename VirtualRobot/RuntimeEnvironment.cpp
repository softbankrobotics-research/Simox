#ifdef WIN32
#   pragma warning (disable:4275) // non dll-interface class 'std::logic_error' used as base for dll-interface class 'boost::program_options::error'
#   pragma warning (disable:4251) // class 'std::vector<_Ty>' needs to have dll-interface to be used by clients of class 'boost::program_options::ambiguous_option'
#   pragma warning (disable:4996) // warning on insecure char* usage, that arises when using boost::split
#endif

#include "RuntimeEnvironment.h"
#include "VirtualRobotException.h"
#include "Visualization/VisualizationFactory.h"


namespace VirtualRobot
{
    bool RuntimeEnvironment::pathInitialized = false;
    
    std::string RuntimeEnvironment::caption = "Simox runtime options";
    
    std::vector< std::pair<std::string, std::string> > RuntimeEnvironment::processKeys;
    std::vector< std::pair<std::string, std::string> > RuntimeEnvironment::processFlags;
    
    std::vector< std::string > RuntimeEnvironment::dataPaths;
    std::vector< std::string > RuntimeEnvironment::unrecognizedOptions;
    std::map< std::string, std::string > RuntimeEnvironment::keyValues;
    std::set< std::string > RuntimeEnvironment::flags;
    bool RuntimeEnvironment::helpFlag = false;

    void RuntimeEnvironment::init()
    {
        if (!pathInitialized)
        {
            pathInitialized = true;
            bool pathFound = false;
            char* simox_data_path = getenv("SIMOX_DATA_PATH");

            if (simox_data_path)
            {
                pathFound = addDataPath(std::string(simox_data_path), true);
            }

            char* vr_data_path = getenv("VIRTUAL_ROBOT_DATA_PATH");

            if (vr_data_path)
            {
                pathFound = addDataPath(std::string(vr_data_path), true);
            }

            if (!pathFound)
            {
                // test for Simox_DIR
                simox_data_path = getenv("Simox_DIR");

                if (simox_data_path)
                {
                    std::string sd(simox_data_path);
                    sd += std::string("/data");
                    pathFound = addDataPath(sd, true);

                    if (!pathFound)
                    {
                        std::string sd(simox_data_path);
                        sd += std::string("/VirtualRobot/data");
                        pathFound = addDataPath(sd, true);
                    }

                    if (!pathFound)
                    {
                        std::string sd(simox_data_path);
                        sd += std::string("../VirtualRobot/data");
                        pathFound = addDataPath(sd, true);
                    }
                }
            }

#ifdef Simox_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(Simox_DATA_PATH), true);
#endif
#ifdef VirtualRobot_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(VirtualRobot_DATA_PATH), true);
#endif
#ifdef VirtualRobot_SRC_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(VirtualRobot_SRC_DATA_PATH), true);
#endif

            // check standard linux install path
            if (!pathFound)
            {
                pathFound = addDataPath(std::string("/usr/local/data"), true);
            }

            if (!pathFound)
            {
                pathFound = addDataPath(std::string("/usr/data"), true);
            }

            // last chance, check for inbuild paths
            if (!pathFound)
            {
                std::filesystem::path p(std::filesystem::current_path());
                std::filesystem::path p1 = std::filesystem::operator/(p, "../VirtualRobot/data");
                std::filesystem::path p2 = std::filesystem::operator/(p, "../../VirtualRobot/data");
                std::filesystem::path p3 = std::filesystem::operator/(p, "../../../VirtualRobot/data");
                std::filesystem::path p4 = std::filesystem::operator/(p, "../../../../VirtualRobot/data");
                pathFound = pathFound | addDataPath(p1.string(), true);
                pathFound = pathFound | addDataPath(p2.string(), true);
                pathFound = pathFound | addDataPath(p3.string(), true);
                pathFound = pathFound | addDataPath(p4.string(), true);
            }
        }
    }
    
    bool RuntimeEnvironment::getDataFileAbsolute(std::string& fileName)
    {
        if (!pathInitialized)
        {
            init();
        }

        std::filesystem::path fn(fileName);

        try
        {
            // first check current path
            if (std::filesystem::exists(fn) && std::filesystem::is_regular_file(fn))
            {
                fileName = fn.string();
                return true;
            }
        }
        catch (...)//const std::filesystem::filesystem_error& /*ex*/)
        {
            //cout << ex.what() << '\n';
            // silently skip this error (e.g. device not ready, permission denied etc)
        }

        for (auto& dataPath : dataPaths)
        {
            std::filesystem::path p(dataPath);

            std::filesystem::path fnComplete = std::filesystem::operator/(p, fn);

            try
            {
                if (std::filesystem::exists(fnComplete) && std::filesystem::is_regular_file(fnComplete))
                {
                    // check for permissions (todo)
                    //std::filesystem::file_status s = std::filesystem::status(fnComplete);
                    //printf("%o\n",s.permissions());

                    fileName = fnComplete.string();
                    return true;
                }
            }
            catch (...)//const std::filesystem::filesystem_error& ex)
            {
                //cout << "EX:" << ex.what() << '\n';
                // silently skip this error (e.g. device not ready, permission denied etc)
            }
        }

        return false;
    }

    void RuntimeEnvironment::processCommandLine(int argc, char* argv[])
    {
        if (!pathInitialized)
        {
            init();
        }
        
        const boost::program_options::options_description description = makeOptionsDescription();
        
        const boost::program_options::parsed_options parsed =
            boost::program_options::command_line_parser(argc, argv).options(description).allow_unregistered().run();

        processParsedOptions(parsed);
    }
    
    boost::program_options::options_description RuntimeEnvironment::makeOptionsDescription()
    {
        // Declare the supported options.
        
        boost::program_options::options_description desc(caption);
        desc.add_options()
            ("help", "Simox command line parser: Set options with '--key value'\n")
            ("data-path", boost::program_options::value<std::vector<std::string>>()->composing(), 
             "Set data path. Multiple data paths are allowed.")
        ;

        for (const auto& item : processKeys)
        {
            desc.add_options()
            (item.first.c_str(), boost::program_options::value<std::vector<std::string>>(), item.second.c_str())
            ;
        }

        for (const auto& item : processFlags)
        {
            desc.add_options()
            (item.first.c_str(), item.second.c_str())
            ;
        }
        return desc;
    }
    
    void RuntimeEnvironment::processParsedOptions(const boost::program_options::parsed_options& parsed)
    {
        boost::program_options::variables_map vm;
        //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::store(parsed, vm);
        boost::program_options::notify(vm);
        
        // process data-path entries
        if (vm.count("data-path"))
        {
            //VR_INFO << "Data paths are: " << endl;
            std::vector<std::string> dp = vm["data-path"].as< std::vector< std::string > >();

            for (const auto& i : dp)
            {
                addDataPath(i);
                //VR_INFO << dp[i] << "\n";
            }
        }

        // process generic keys
        for (const auto& processKey : processKeys)
        {
            const std::string& key = processKey.first;
            if (vm.count(key.c_str()) > 0)
            {
                std::vector<std::string> dp = vm[key.c_str()].as<std::vector<std::string>>();

                if (dp.size() > 1)
                {
                    VR_WARNING << "More than one parameter for key '" << key << "'. Using only first one..." << endl;
                }

                if (dp.size() > 0)
                {
                    addKeyValuePair(key, dp[0]);    // take the first one...
                }
            }

        }
        
        for (const auto& flag : processFlags)
        {
            if (vm.count(flag.first.c_str()) > 0)
            {
                flags.insert(flag.first);
            }
        }
        helpFlag = vm.count("help") > 0;

        // collect unrecognized arguments
        std::vector<std::string> options = boost::program_options::collect_unrecognized(
                    parsed.options, boost::program_options::include_positional);

        for (const auto& option : options)
        {
            unrecognizedOptions.push_back(option);
        }
    }
    
    void RuntimeEnvironment::addKeyValuePair(const std::string& key, const std::string& value)
    {
        keyValues[key] = value;
    }

    std::string RuntimeEnvironment::getValue(const std::string& key, const std::string& defaultValue)
    {
        if (keyValues.find(key) != keyValues.end())
        {
            return keyValues[key];
        }

        return defaultValue;
    }

    std::map< std::string, std::string > RuntimeEnvironment::getKeyValuePairs()
    {
        return keyValues;
    }

    std::vector< std::string> RuntimeEnvironment::getUnrecognizedOptions()
    {
        return unrecognizedOptions;
    }


    std::vector< std::string > RuntimeEnvironment::getDataPaths()
    {
        if (!pathInitialized)
        {
            init();
        }

        return dataPaths;
    }
    
    void RuntimeEnvironment::setCaption(const std::string& caption)
    {
        RuntimeEnvironment::caption = caption;
    }

    bool RuntimeEnvironment::addDataPath(const std::string& path, bool quiet)
    {
        try
        {
            std::filesystem::path p(path);

            if (std::filesystem::is_directory(p) || std::filesystem::is_symlink(p))
            {
                dataPaths.push_back(path);
                return true;
            }
        }
        catch (...)
        {
        }

        if (!quiet)
        {
            VR_ERROR << "Trying to add non-existing data path: " << path << endl;
        }

        return false;
    }

    static std::size_t getMaxLength(const std::vector<std::pair<std::string, std::string>>& items)
    {
        std::size_t max = 0;
        for (const auto& key : items)
        {
            max = std::max(max, key.first.size());
        }
        return max;
    }
    
    static std::string padding(std::size_t current, std::size_t target, char c = ' ')
    {
        std::stringstream ss;
        while (target --> current)
        {
            ss << c;
        }
        return ss.str();
    }
    
    void RuntimeEnvironment::print()
    {
        if (!pathInitialized)
        {
            init();
        }

        cout << " *********** Simox RuntimeEnvironment ************* " << endl;
        cout << "Data paths:"  << endl;

        for (const auto& dataPath : dataPaths)
        {
            cout << " * " << dataPath << endl;
        }

        const std::size_t descriptionOffset = std::max(
                    getMaxLength(processKeys), getMaxLength(processFlags)) + 4;
        
        auto printDescriptions = [&descriptionOffset](
                const std::vector<std::pair<std::string, std::string>>& items,
                const std::string& name)
        {
            if (items.size() > 0)
            {
                cout << "Known " << name << ":" << endl;
    
                for (const auto& item : items)
                {
                    cout << " * " << item.first
                         << padding(item.first.size(), descriptionOffset)
                         << item.second << endl;
                }
            }
        };
        
        printDescriptions(processKeys, "keys");
        printDescriptions(processFlags, "flags");
        

        if (keyValues.size() > 0)
        {
            cout << "Parsed options:"  << endl;
            for (const auto& item : keyValues)
            {
                cout << " * " << item.first << ": " << item.second << endl;
            }
        }
        
        if (flags.size() > 0)
        {
            cout << "Parsed flags:"  << endl;
            for (const auto& flag : flags)
            {
                cout << " * " << flag << endl;
            }
        }

        if (unrecognizedOptions.size() > 0)
        {
            cout << "Unrecognized options:" << endl;

            for (const auto& unrecognizedOption : unrecognizedOptions)
            {
                cout << " * <" << unrecognizedOption << ">" << endl;
            }
        }
    }
    
    void RuntimeEnvironment::printOptions(std::ostream& os)
    {
        os << makeOptionsDescription() << std::endl;
    }

    void RuntimeEnvironment::considerKey(const std::string& key, const std::string& description)
    {
        processKeys.emplace_back(key, description);
    }
    
    void RuntimeEnvironment::considerFlag(const std::string& flag, const std::string& description)
    {
        processFlags.emplace_back(flag, description);
    }

    bool RuntimeEnvironment::hasValue(const std::string& key)
    {
        return keyValues.find(key) != keyValues.end();
    }
    
    bool RuntimeEnvironment::hasFlag(const std::string& flag)
    {
        return flags.find(flag) != flags.end();
    }
    
    bool RuntimeEnvironment::hasHelpFlag()
    {
        return helpFlag;
    }


    float RuntimeEnvironment::toFloat(const std::string& s)
    {
        return std::stof(s);
    }

    int RuntimeEnvironment::toInt(const std::string& s)
    {
        return std::stoi(s);
    }

    bool RuntimeEnvironment::toVector3f(const std::string& string, Eigen::Vector3f& storeResult)
    {
        if (string.length() < 3)
        {
            return false;
        }

        if (string[0] != '(' || string[string.length() - 1] != ')')
        {
            VR_WARNING << "Expecting string to start and end with brackets (): " << string << endl;
            return false;
        }

        const std::string stringTrimmed = string.substr(1, string.size() - 1);
        const std::string delimiter = ",";

        std::vector<std::string> stringSplit;
        boost::split(stringSplit, stringTrimmed, boost::is_any_of(delimiter));

        if (stringSplit.size() != 3)
        {
            VR_WARNING << "Expecting values of string to be separated with a ',': " << string << endl;
            return false;
        }

        Eigen::Vector3f result;
        
        for (int i = 0; i < result.SizeAtCompileTime; ++i)
        {
            const std::string& string = stringSplit[static_cast<std::size_t>(i)];
            bool error = false;
            float a;
            try
            {
                a = std::stof(string);
            }
            catch (const std::invalid_argument&)
            {
                error = true;
            }
            if (error || boost::math::isinf(a) || boost::math::isinf(-a) || boost::math::isnan(a))
            {
                VR_WARNING << "Could not convert '" << string << "' to a number." << endl;
                return false;
            }
            result(i) = a;
        }
        
        storeResult = result;
        return true;
    }

    std::string RuntimeEnvironment::checkValidFileParameter(const std::string& key, const std::string& standardFilename)
    {
        if (VirtualRobot::RuntimeEnvironment::hasValue(key))
        {
            std::string f = RuntimeEnvironment::getValue(key);

            if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(f))
            {
                return f;
            }
        }

        // don't check for empty files
        if (standardFilename.empty())
        {
            return standardFilename;
        }

        std::string s = standardFilename;

        if (!RuntimeEnvironment::getDataFileAbsolute(s))
        {
            VR_WARNING << "Could not determine path to file " << standardFilename << endl;
            return standardFilename;
        }

        return s;
    }

    std::string RuntimeEnvironment::checkParameter(const std::string& key, const std::string& standardValue /*= ""*/)
    {
        if (RuntimeEnvironment::hasValue(key))
        {
            return RuntimeEnvironment::getValue(key);
        }

        return standardValue;
    }

    void RuntimeEnvironment::cleanup()
    {
        VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(nullptr);

        if (visualizationFactory)
        {
            visualizationFactory->cleanup();
        }
    }




} //  namespace


