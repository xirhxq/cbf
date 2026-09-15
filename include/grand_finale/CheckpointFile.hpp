#pragma once
#include "grand_finale/CheckpointCodec.hpp"
#include <filesystem>
#include <fstream>
#include <limits>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>
#if defined(__APPLE__)
#include <CommonCrypto/CommonDigest.h>
#else
#include <openssl/sha.h>
#endif

namespace gf {
class CheckpointFile {
    using Json=checkpoint::Json;
public:
    static std::array<unsigned char,32> digest(const unsigned char* data,std::size_t size){
        std::array<unsigned char,32> out{};
#if defined(__APPLE__)
        if(size>std::numeric_limits<CC_LONG>::max())throw std::invalid_argument("checkpoint hash size");
        CC_SHA256(data,static_cast<CC_LONG>(size),out.data());
#else
        SHA256(data,size,out.data());
#endif
        return out;
    }
    static std::string sha256(const std::vector<unsigned char>& data){
        const auto d=digest(data.data(),data.size());std::string out;constexpr char h[]="0123456789abcdef";
        for(auto c:d){out.push_back(h[c>>4]);out.push_back(h[c&15]);}return out;
    }
    static std::string fileSha256(const std::filesystem::path& p){std::ifstream in(p,std::ios::binary);if(!in)throw std::runtime_error("cannot hash checkpoint dependency");return sha256({std::istreambuf_iterator<char>(in),{}});}
    static Json read(const std::filesystem::path& path){
        gzFile in=gzopen(path.c_str(),"rb");if(!in)throw std::runtime_error("cannot open checkpoint");
        std::vector<unsigned char> bytes;std::array<unsigned char,65536> buffer{};
        try{for(;;){const int n=gzread(in,buffer.data(),buffer.size());if(n<0)throw std::runtime_error("checkpoint gzip corruption");if(n==0)break;
                if(bytes.size()+n>128U*1024U*1024U)throw std::invalid_argument("checkpoint exceeds 128 MiB limit");bytes.insert(bytes.end(),buffer.begin(),buffer.begin()+n);}
            const int result=gzclose(in);in=nullptr;if(result!=Z_OK)throw std::runtime_error("checkpoint close/integrity error");
        }catch(...){if(in)gzclose(in);throw;}
        constexpr char magic[]="GFCKPT01";
        if(bytes.size()<48||std::memcmp(bytes.data(),magic,8)!=0)throw std::invalid_argument("checkpoint file schema");
        std::uint64_t count=0;for(int k=0;k<8;++k)count|=std::uint64_t(bytes[8+k])<<(8*k);
        if(count!=bytes.size()-48)throw std::invalid_argument("checkpoint length mismatch");
        const auto hash=digest(bytes.data()+48,count);
        if(!std::equal(hash.begin(),hash.end(),bytes.begin()+16))throw std::invalid_argument("checkpoint SHA-256 mismatch");
        return Json::from_cbor(bytes.begin()+48,bytes.end(),true,true);
    }
    static void writeNew(const std::filesystem::path& path,const Json& value){
        if(!path.is_absolute()||!std::filesystem::is_directory(path.parent_path()))throw std::invalid_argument("checkpoint needs existing absolute output directory");
        if(std::filesystem::exists(path))throw std::invalid_argument("checkpoint destination exists");
        const auto payload=Json::to_cbor(value);
        if(payload.size()>128U*1024U*1024U-48)throw std::invalid_argument("checkpoint exceeds file size contract");
        const auto hash=digest(payload.data(),payload.size());
        std::vector<unsigned char> bytes{'G','F','C','K','P','T','0','1'};
        for(int k=0;k<8;++k)bytes.push_back((std::uint64_t(payload.size())>>(8*k))&255);
        bytes.insert(bytes.end(),hash.begin(),hash.end());bytes.insert(bytes.end(),payload.begin(),payload.end());
        std::string temporary=path.string()+".partial-XXXXXX";std::vector<char> name(temporary.begin(),temporary.end());name.push_back('\0');
        const int fd=::mkstemp(name.data());if(fd<0)throw std::runtime_error("cannot create checkpoint temporary");
        temporary=name.data();gzFile out=gzdopen(fd,"wb6");if(!out){::close(fd);::unlink(temporary.c_str());throw std::runtime_error("cannot compress checkpoint");}
        try{if(gzwrite(out,bytes.data(),static_cast<unsigned>(bytes.size()))!=static_cast<int>(bytes.size()))throw std::runtime_error("checkpoint write failed");
            const int status=gzclose(out);out=nullptr;if(status!=Z_OK)throw std::runtime_error("checkpoint flush failed");
            const int syncfd=::open(temporary.c_str(),O_RDONLY);if(syncfd<0)throw std::runtime_error("checkpoint sync open");const int synced=::fsync(syncfd);::close(syncfd);if(synced!=0)throw std::runtime_error("checkpoint sync failed");
            if(read(temporary)!=value)throw std::runtime_error("checkpoint independent readback failed");
            // link is atomic and refuses an existing destination, unlike rename.
            if(::link(temporary.c_str(),path.c_str())!=0)throw std::runtime_error("checkpoint atomic publication failed");
            ::unlink(temporary.c_str());
            syncDirectory(path.parent_path());
        }catch(...){if(out)gzclose(out);::unlink(temporary.c_str());throw;}
    }
    static void syncDirectory(const std::filesystem::path& directory){
        const int fd=::open(directory.c_str(),O_RDONLY);if(fd<0)throw std::runtime_error("checkpoint directory sync open");
        const int status=::fsync(fd);::close(fd);if(status!=0)throw std::runtime_error("checkpoint directory sync failed");
    }
    static void writeJsonNew(const std::filesystem::path& path,const Json& value){
        if(!path.is_absolute()||!std::filesystem::is_directory(path.parent_path())||std::filesystem::exists(path))
            throw std::invalid_argument("checkpoint receipt requires a new absolute path");
        const auto bytes=value.dump(2)+"\n";std::string temp=path.string()+".partial-XXXXXX";
        std::vector<char> name(temp.begin(),temp.end());name.push_back('\0');int fd=::mkstemp(name.data());
        if(fd<0)throw std::runtime_error("checkpoint receipt temporary");temp=name.data();
        try{std::size_t written=0;while(written<bytes.size()) {const auto n=::write(fd,bytes.data()+written,bytes.size()-written);if(n<=0)throw std::runtime_error("checkpoint receipt write");written+=n;}
            if(::fsync(fd)!=0)throw std::runtime_error("checkpoint receipt sync");::close(fd);fd=-1;
            if(::link(temp.c_str(),path.c_str())!=0)throw std::runtime_error("checkpoint receipt publication");::unlink(temp.c_str());syncDirectory(path.parent_path());
        }catch(...){if(fd>=0)::close(fd);::unlink(temp.c_str());throw;}
    }
};
} // namespace gf
