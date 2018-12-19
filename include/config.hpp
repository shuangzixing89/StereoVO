//
// Created by lixin on 18-12-19.
//

#ifndef STEREOVO_CONFIG_HPP
#define STEREOVO_CONFIG_HPP

#include "common.hpp"

namespace StereoVO
{
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config () {} // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static void setParameterFile( const std::string& filename );

        // access the parameter values
        template< typename T >
        static T get( const std::string& key )
        {
            return T( Config::config_->file_[key] );
        }
    };
}


#endif //STEREOVO_CONFIG_HPP
