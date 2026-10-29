// live_feed.cpp

#include "receiver/app/live_feed.h"

#include <utility>

namespace spectagatr::receiver::app
{

namespace
{

constexpr int kPosePriority   = 0;
constexpr int kStatusPriority = 1;
constexpr int kRobotPriority  = 1;
constexpr int kPathPriority   = 2;

feed::RxMillis millisOf(sources::ReceiverUs us) { return static_cast<double>(us) / 1000.0; }

} // namespace

feed::LinkState linkStateOf(sources::PortState state) {
    switch (state) {
    case sources::PortState::kOpen:
        return feed::LinkState::kOpen;
    case sources::PortState::kClosed:
        return feed::LinkState::kError;
    case sources::PortState::kSearching:
    case sources::PortState::kNoBrainLinked:
    case sources::PortState::kStopped:
        return feed::LinkState::kSearching;
    }
    return feed::LinkState::kSearching;
}

std::string detailOf(const sources::SourceStatus& status) {
    std::string detail = status.detail;
    if (status.state == sources::PortState::kClosed) {
        detail = "lost: " + detail;
    }
    if (status.src == sources::SourceKind::kController &&
        status.state == sources::PortState::kOpen && status.stats.radio) {
        detail += ", radio quality " + std::to_string(status.stats.radio->quality);
    }
    return detail;
}

LiveFeed::LiveFeed(server::HttpServer& server, FeedSettings settings)
    : server_(server), settings_(std::move(settings)) {}

void LiveFeed::onChunk(const sources::SourceChunk& chunk) {
    const std::lock_guard<std::mutex> lock(mutex_);
    const char* const                 source = sources::sourceKindName(chunk.src);
    const feed::AuxChange             change = feed_.push(
        chunk.bytes.data(), chunk.bytes.size(), millisOf(chunk.rx_us),
        [&](const feed::TrackedPose& pose) {
            std::string                     json  = feed::poseJson(pose, source);
            const server::HttpServer::Frame frame = server::HttpServer::textFrame(json);
            for (const ClientId client : clients_) {
                if (pose.restart) {
                    // In order, after nothing older: the trail break always arrives.
                    server_.clearLatest(client, kPoseChannel);
                    server_.sendReliable(client, frame);
                } else {
                    server_.sendLatest(client, kPoseChannel, kPosePriority, frame);
                }
            }
            last_pose_ = std::move(json);
        });
    if (change.robot) {
        const server::HttpServer::Frame frame = server::HttpServer::textFrame(robotLocked());
        for (const ClientId client : clients_) {
            server_.sendLatest(client, kRobotChannel, kRobotPriority, frame);
        }
    }
    if (change.path) {
        path_dirty_ = true; // whole path once per tick, not once per point
    }
}

void LiveFeed::onStatus(const sources::SourceStatus& status) {
    const std::lock_guard<std::mutex> lock(mutex_);
    const feed::LinkState             link = linkStateOf(status.state);
    if (link != link_) {
        feed_.resetTransport(); // a partial line from before is not continued
    }
    if (status.stream_restarted) {
        feed_.resetTransport();
        feed_.forgetOrdering();
    }
    link_   = link;
    source_ = sources::sourceKindName(status.src);
    port_   = status.port;
    detail_ = detailOf(status);
}

void LiveFeed::onOpen(ClientId client) {
    const std::lock_guard<std::mutex> lock(mutex_);
    server_.sendReliable(client, server::HttpServer::textFrame(feed::helloJson(settings_.hello)));
    if (feed_.robot()) {
        server_.sendLatest(client, kRobotChannel, kRobotPriority,
                           server::HttpServer::textFrame(robotLocked()));
    }
    if (feed_.path().hasPath()) {
        server_.sendLatest(client, kPathChannel, kPathPriority,
                           server::HttpServer::textFrame(feed::pathJson(feed_.path())));
    }
    if (sent_) {
        server_.sendLatest(client, kStatusChannel, kStatusPriority,
                           server::HttpServer::textFrame(feed::statusJson(*sent_)));
    }
    if (last_pose_) {
        server_.sendLatest(client, kPoseChannel, kPosePriority,
                           server::HttpServer::textFrame(*last_pose_));
    }
    clients_.insert(client);
}

void LiveFeed::onClose(ClientId client) {
    const std::lock_guard<std::mutex> lock(mutex_);
    clients_.erase(client);
}

feed::FeedStatus LiveFeed::tick(feed::RxMillis now) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (path_dirty_) {
        path_dirty_                           = false;
        const server::HttpServer::Frame frame = server::HttpServer::textFrame(feed::pathJson(feed_.path()));
        for (const ClientId client : clients_) {
            server_.sendLatest(client, kPathChannel, kPathPriority, frame);
        }
    }
    const feed::FeedStatus status = statusLocked(now);
    if (!sent_ || feed::statusChanged(*sent_, status) || now - sent_at_ >= kStatusPeriodMs) {
        sendStatusLocked(status);
        sent_    = status;
        sent_at_ = now;
    }
    return status;
}

feed::FeedStatus LiveFeed::status(feed::RxMillis now) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return statusLocked(now);
}

std::string LiveFeed::robotMessage() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return robotLocked();
}

std::string LiveFeed::pathMessage() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return feed::pathJson(feed_.path());
}

std::string LiveFeed::robotLocked() const {
    const std::optional<protocol::RobotRecord>& robot = feed_.robot();
    return feed::robotJson(robot ? &*robot : nullptr, settings_.hello.robot);
}

feed::FeedStatus LiveFeed::statusLocked(feed::RxMillis now) const {
    feed::FeedStatus status;
    status.rx_ms       = now;
    status.source      = source_;
    status.port        = port_;
    status.link        = link_;
    status.detail      = detail_;
    status.stale       = feed::isStale(feed_.lastRx(), now, settings_.stale_ms, link_);
    status.pose_age_ms = feed::poseAgeMs(feed_.lastRx(), now);
    status.counts      = feed_.counts();
    return status;
}

void LiveFeed::sendStatusLocked(const feed::FeedStatus& status) {
    const server::HttpServer::Frame frame = server::HttpServer::textFrame(feed::statusJson(status));
    for (const ClientId client : clients_) {
        server_.sendLatest(client, kStatusChannel, kStatusPriority, frame);
    }
}

} // namespace spectagatr::receiver::app
