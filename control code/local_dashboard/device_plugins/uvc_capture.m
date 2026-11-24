#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>
#import <mach/mach_time.h>
#import <signal.h>
#import <stdio.h>
#import <string.h>
#import <unistd.h>

static NSArray<AVCaptureDevice *> *videoDevices(void) {
    NSArray<AVCaptureDeviceType> *types;
    if (@available(macOS 14.0, *)) {
        types = @[AVCaptureDeviceTypeBuiltInWideAngleCamera, AVCaptureDeviceTypeExternal,
                  AVCaptureDeviceTypeContinuityCamera];
    } else {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
        types = @[AVCaptureDeviceTypeBuiltInWideAngleCamera, AVCaptureDeviceTypeExternalUnknown];
#pragma clang diagnostic pop
    }
    return [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:types
                                                                  mediaType:AVMediaTypeVideo
                                                                   position:AVCaptureDevicePositionUnspecified].devices;
}

static NSString *cameraType(AVCaptureDevice *device) {
    if (@available(macOS 14.0, *)) {
        if ([device.deviceType isEqualToString:AVCaptureDeviceTypeContinuityCamera]) return @"continuity";
    }
    return [device.deviceType isEqualToString:AVCaptureDeviceTypeBuiltInWideAngleCamera]
        ? @"built_in" : @"external";
}

static BOOL writeJSON(id object) {
    NSData *data = [NSJSONSerialization dataWithJSONObject:object options:0 error:NULL];
    return data && fwrite(data.bytes, 1, data.length, stdout) == data.length && fputc('\n', stdout) != EOF;
}

static int fail(NSString *message) {
    fprintf(stderr, "%s\n", message.UTF8String);
    return 1;
}

static double hostSeconds(void) {
    static mach_timebase_info_data_t scale;
    if (scale.denom == 0) mach_timebase_info(&scale);
    return (double)mach_absolute_time() * scale.numer / scale.denom / 1e9;
}

static double negotiatedFPS(AVCaptureDevice *device) {
    double period = CMTimeGetSeconds(device.activeVideoMinFrameDuration);
    return isfinite(period) && period > 0 ? 1 / period : 0;
}

static void selectFormat(AVCaptureDevice *device) {
    // Pick the supported 30 fps mode closest to 1080p. Auto configuration
    // remains available for devices whose only modes use other frame rates.
    AVCaptureDeviceFormat *best = nil;
    double bestScore = INFINITY;
    CMTime duration = CMTimeMake(1, 30), bestDuration = duration;
    for (AVCaptureDeviceFormat *format in device.formats) {
        CMVideoDimensions size = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
        if (size.width <= 0 || size.height <= 0) continue;
        for (AVFrameRateRange *range in format.videoSupportedFrameRateRanges) {
            // 29.97 fps is commonly advertised as the nominal 30 fps mode.
            if (range.minFrameRate > 30.05 || range.maxFrameRate < 29.95) continue;
            CMTime actual = CMTimeMaximum(range.minFrameDuration, CMTimeMinimum(duration, range.maxFrameDuration));
            double fps = 1 / CMTimeGetSeconds(actual);
            double score = fabs(log(size.width / 1920.0)) + fabs(log(size.height / 1080.0))
                         + fabs(log((size.width / (double)size.height) / (16.0 / 9))) + .01*fabs(fps-30);
            if (score < bestScore) {
                best = format;
                bestScore = score;
                bestDuration = actual;
            }
        }
    }
    if (!best || ![device lockForConfiguration:NULL]) return;
    device.activeFormat = best;
    device.activeVideoMinFrameDuration = bestDuration;
    device.activeVideoMaxFrameDuration = bestDuration;
    [device unlockForConfiguration];
}

@interface Frames : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property(nonatomic) double last;
@property(nonatomic) NSUInteger sequence;
@property(nonatomic, strong) AVCaptureDevice *device;
@end

@implementation Frames
- (void)captureOutput:(AVCaptureOutput *)output
 didOutputSampleBuffer:(CMSampleBufferRef)sample
       fromConnection:(AVCaptureConnection *)connection {
    (void)output;
    (void)connection;
    double stamp = hostSeconds();
    // Limit IPC bandwidth independently of the camera's negotiated mode.
    if (stamp - self.last < .095) return;
    CVPixelBufferRef pixels = CMSampleBufferGetImageBuffer(sample);
    if (!pixels || CVPixelBufferLockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly) != kCVReturnSuccess) return;
    size_t width = CVPixelBufferGetWidth(pixels), height = CVPixelBufferGetHeight(pixels);
    size_t stride = CVPixelBufferGetBytesPerRow(pixels), rowBytes = width * 4;
    const unsigned char *base = CVPixelBufferGetBaseAddress(pixels);
    double fps = negotiatedFPS(self.device);
    double period = CMTimeGetSeconds(CMSampleBufferGetDuration(sample));
    if (fps <= 0 && isfinite(period) && period > 0) fps = 1 / period;
    BOOL ok = base && stride >= rowBytes && writeJSON(@{
        @"host_receive_monotonic": @(stamp), @"frame_number": @(++self.sequence),
        @"width": @(width), @"height": @(height), @"fps": @(fps), @"bytes": @(rowBytes * height)
    });
    for (size_t y = 0; ok && y < height; ++y) {
        ok = fwrite(base + y * stride, 1, rowBytes, stdout) == rowBytes;
    }
    CVPixelBufferUnlockBaseAddress(pixels, kCVPixelBufferLock_ReadOnly);
    if (!ok || fflush(stdout) != 0) _exit(1);
    self.last = stamp;
}
@end

static int capture(NSString *identifier) {
    AVCaptureDevice *device = nil;
    for (AVCaptureDevice *candidate in videoDevices()) {
        if ([candidate.uniqueID isEqualToString:identifier]) {
            device = candidate;
            break;
        }
    }
    if (!device) return fail(@"所选摄像头已离线，请重新刷新设备列表");
    AVAuthorizationStatus status = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    if (status == AVAuthorizationStatusNotDetermined) {
        dispatch_semaphore_t pending = dispatch_semaphore_create(0);
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
            (void)granted;
            dispatch_semaphore_signal(pending);
        }];
        dispatch_semaphore_wait(pending, DISPATCH_TIME_FOREVER);
        status = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    }
    if (status != AVAuthorizationStatusAuthorized) {
        return fail(@"摄像头权限未授予，请在系统设置 → 隐私与安全性 → 摄像头中允许访问");
    }
    NSError *error = nil;
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if (!input) return fail(error.localizedDescription ?: @"无法打开所选摄像头");
    __attribute__((objc_precise_lifetime)) AVCaptureSession *session = [AVCaptureSession new];
    AVCaptureVideoDataOutput *output = [AVCaptureVideoDataOutput new];
    __attribute__((objc_precise_lifetime)) Frames *frames = [Frames new];
    frames.device = device;
    output.videoSettings = @{ (id)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA) };
    output.alwaysDiscardsLateVideoFrames = YES;
    [output setSampleBufferDelegate:frames queue:dispatch_queue_create("org.ophis.uvc.frames", DISPATCH_QUEUE_SERIAL)];
    [session beginConfiguration];
    if (![session canAddInput:input] || ![session canAddOutput:output]) return fail(@"无法配置侧视视频流");
    [session addInput:input];
    [session addOutput:output];
    selectFormat(device);
    AVCaptureConnection *connection = [output connectionWithMediaType:AVMediaTypeVideo];
    if (connection.isVideoMirroringSupported) {
        connection.automaticallyAdjustsVideoMirroring = NO;
        connection.videoMirrored = NO;
    }
    [session commitConfiguration];
    NSNotificationCenter *notifications = NSNotificationCenter.defaultCenter;
    [notifications addObserverForName:AVCaptureSessionRuntimeErrorNotification object:session queue:nil
                          usingBlock:^(NSNotification *notification) {
        NSError *failure = notification.userInfo[AVCaptureSessionErrorKey];
        fail(failure.localizedDescription ?: @"侧视摄像头视频流出错");
        _exit(1);
    }];
    [notifications addObserverForName:AVCaptureDeviceWasDisconnectedNotification object:device queue:nil
                          usingBlock:^(NSNotification *notification) {
        (void)notification;
        fail(@"侧视摄像头已断开");
        _exit(1);
    }];
    [session startRunning];
    if (!session.isRunning) return fail(@"侧视摄像头无法开始采集");
    // Keep the capture session and delegate alive until the parent stops the helper.
    dispatch_semaphore_wait(dispatch_semaphore_create(0), DISPATCH_TIME_FOREVER);
    return 0;
}

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        signal(SIGPIPE, SIG_IGN);
        if (argc == 2 && strcmp(argv[1], "--list") == 0) {
            NSMutableArray *devices = [NSMutableArray array];
            for (AVCaptureDevice *device in videoDevices()) {
                [devices addObject:@{@"id": device.uniqueID, @"name": device.localizedName, @"type": cameraType(device)}];
            }
            return writeJSON(devices) ? 0 : 1;
        }
        if (argc == 3 && strcmp(argv[1], "--capture") == 0) {
            return capture([NSString stringWithUTF8String:argv[2]]);
        }
        return fail(@"Usage: ophis-uvc-capture --list | --capture DEVICE_ID");
    }
}
