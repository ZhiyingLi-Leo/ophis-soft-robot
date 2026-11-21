// macOS D405 transport. Open the requested camera and drop privileges before IPC.
#include <librealsense2/rs.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <grp.h>
#include <sys/stat.h>
#include <signal.h>
#include <errno.h>
#include <pwd.h>
#include <unistd.h>
#include <mach/mach_time.h>
#include <chrono>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

template<class T> void array_json(std::ostream &out, const T &values) {
    out << '[';
    bool first = true;
    for (auto value : values) {
        if (!first) out << ',';
        out << value; first = false;
    }
    out << ']';
}
void intrinsics(std::ostream &out, rs2::stream_profile profile) {
    auto i = profile.as<rs2::video_stream_profile>().get_intrinsics();
    out << "{\"width\":" << i.width << ",\"height\":" << i.height
              << ",\"fx\":" << i.fx << ",\"fy\":" << i.fy << ",\"ppx\":" << i.ppx
              << ",\"ppy\":" << i.ppy << ",\"model_id\":" << int(i.model) << ",\"coeffs\":";
    array_json(out,i.coeffs); out << '}';
}
bool send_all(int fd,const void *data,size_t bytes) {
    auto ptr=static_cast<const char*>(data);
    while(bytes) {
        auto n=send(fd,ptr,bytes,0);
        if(n<=0) return false;
        ptr+=n; bytes-=n;
    }
    return true;
}
bool pixels(int fd,rs2::video_frame frame) {
    int row=frame.get_width()*frame.get_bytes_per_pixel();
    auto data=static_cast<const char*>(frame.get_data());
    if(frame.get_stride_in_bytes()==row)
        return send_all(fd,data,row*frame.get_height());
    for(int y=0;y<frame.get_height();++y)
        if(!send_all(fd,data+y*frame.get_stride_in_bytes(),row)) return false;
    return true;
}
rs2::pipeline_profile start_camera(rs2::pipeline &pipe, rs2::context &context, rs2::config &config) {
    for (int attempt = 1;; ++attempt) {
        try {
            return pipe.start(config);
        } catch (const rs2::error &error) {
            if (std::string(error.what()).find("failed to set power state") == std::string::npos) throw;
            if (attempt == 3)
                throw std::runtime_error("D405 USB 接口无法启动 (failed to set power state)。请结束占用相机的应用后重试。");
            pipe = rs2::pipeline(context);
            std::cerr << "D405 USB 接口尚未就绪，正在重试 (" << attempt + 1 << "/3)\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
        }
    }
}
int main(int argc, char **argv) try {
    if (argc > 2 || !getenv("SUDO_UID")) throw std::runtime_error("Run from Terminal using sudo");
    auto user = getpwuid(std::stoul(getenv("SUDO_UID")));
    if (!user || user->pw_uid == 0) return 2;
    rs2::context ctx;
    std::string serial;
    for (auto dev : ctx.query_devices()) {
        if (std::string(dev.get_info(RS2_CAMERA_INFO_NAME)).find("D405")==std::string::npos) continue;
        std::string id=dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (argc==2 && id!=argv[1]) continue;
        if (!serial.empty()) throw std::runtime_error("Multiple D405 cameras: specify serial");
        serial=id;
    }
    if (serial.empty()) throw std::runtime_error("D405 not found");
    rs2::pipeline pipe(ctx); rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    auto profile=start_camera(pipe,ctx,cfg);
    if (setgroups(0,nullptr) || setgid(user->pw_gid) || setuid(user->pw_uid) || geteuid()==0)
        throw std::runtime_error("Cannot drop camera process privileges");
    std::string folder="/tmp/ophis-d405-"+std::to_string(getuid());
    struct stat dir{};
    if ((mkdir(folder.c_str(),0700) && errno!=EEXIST) || lstat(folder.c_str(),&dir)
        || !S_ISDIR(dir.st_mode) || dir.st_uid!=getuid() || (dir.st_mode & 077))
        throw std::runtime_error("Invalid private camera directory");
    std::string path=folder+"/frames";
    sockaddr_un address{}; address.sun_family=AF_UNIX;
    strlcpy(address.sun_path,path.c_str(),sizeof(address.sun_path));
    int server=socket(AF_UNIX,SOCK_STREAM,0);
    // The pipeline is already exclusively owned by this process.
    unlink(path.c_str());
    if(server<0 || bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address)) || listen(server,1))
        throw std::runtime_error("Cannot open camera socket");
    signal(SIGPIPE,SIG_IGN);
    std::cerr << "D405 ready: click Connect in the dashboard. Ctrl-C closes the camera." << std::endl;
    for (;;) {
        int fd=accept(server,nullptr,nullptr);
        if(fd<0) throw std::runtime_error("Camera accept failed");
        std::ostringstream out;
        auto device=profile.get_device();
        auto depth=profile.get_stream(RS2_STREAM_DEPTH),color=profile.get_stream(RS2_STREAM_COLOR);
        auto extr=depth.get_extrinsics_to(color);
        out << std::setprecision(12) << "{\"serial\":" << std::quoted(serial)
                  << ",\"firmware\":" << std::quoted(device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
                  << ",\"stream_fps\":30,\"depth_scale_m\":" << device.first<rs2::depth_sensor>().get_depth_scale()
                  << ",\"depth\":"; intrinsics(out,depth); out << ",\"color\":"; intrinsics(out,color);
        out << ",\"depth_to_color\":{\"rotation\":"; array_json(out,extr.rotation);
        out << ",\"translation_m\":"; array_json(out,extr.translation); out << "}}" << std::endl;
        rs2::align align(RS2_STREAM_COLOR);
        auto header=out.str();
        mach_timebase_info_data_t timebase{}; mach_timebase_info(&timebase);
        if(send_all(fd,header.data(),header.size())) while (true) {
            out.str(""); out.clear();
            auto frames=pipe.wait_for_frames(3000);
            const auto stamp=double(mach_absolute_time()) * timebase.numer / timebase.denom / 1e9;
            auto d=frames.get_depth_frame(); auto c=frames.get_color_frame();
            auto aligned=align.process(frames).as<rs2::frameset>().get_depth_frame();
            out << "{\"host_receive_monotonic\":" << stamp << ",\"frame_number\":" << d.get_frame_number()
                      << ",\"color_frame_number\":" << c.get_frame_number() << ",\"depth_timestamp_ms\":" << d.get_timestamp()
                      << ",\"color_timestamp_ms\":" << c.get_timestamp() << ",\"depth_clock\":" << std::quoted(rs2_timestamp_domain_to_string(d.get_frame_timestamp_domain()))
                      << ",\"color_clock\":" << std::quoted(rs2_timestamp_domain_to_string(c.get_frame_timestamp_domain())) << "}\n";
            auto meta=out.str();
            if(!send_all(fd,meta.data(),meta.size()) || !pixels(fd,c) || !pixels(fd,d) || !pixels(fd,aligned)) break;
            usleep(140000);
        }
        close(fd);
    }
    return 0;
} catch(const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
