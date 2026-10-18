#include <KeyFrame.h>
#include <KeyFrameDatabase.h>
#include <Map.h>
#include <ORBVocabulary.h>
#include <spdlog/sinks/null_sink.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

class TestKeyFrame : public ORB_SLAM3::KeyFrame
{
public:
    TestKeyFrame(unsigned long id, ORB_SLAM3::Map& map, double shared_weight,
                 bool bad = false)
    {
        mnId = id;
        UpdateMap(&map);
        mbBad = bad;
        // Normalized two-word vectors give distinct L1 scores against word 0.
        mBowVec[0] = shared_weight;
        if (shared_weight < 1.0) mBowVec[1] = 1.0 - shared_weight;
    }
};

class TestDatabase : public ORB_SLAM3::KeyFrameDatabase
{
public:
    explicit TestDatabase(const ORB_SLAM3::ORBVocabulary& vocabulary)
        : ORB_SLAM3::KeyFrameDatabase(vocabulary)
    {
        // Prepared BoW bypasses vocabulary training/loading. The real scorer,
        // add() and DetectNBestCandidates() only need these two word buckets.
        mvInvertedFile.resize(2);
    }
};

struct Fixture
{
    ORB_SLAM3::ORBVocabulary vocabulary;
    TestDatabase database{vocabulary};
    ORB_SLAM3::Map current_map;
    ORB_SLAM3::Map merge_map;
    TestKeyFrame query;
    std::vector<std::unique_ptr<TestKeyFrame>> candidates;

    explicit Fixture(const std::shared_ptr<spdlog::logger>& logger)
        : current_map(0, logger), merge_map(0, logger), query(100, current_map, 1.0)
    {
    }

    ORB_SLAM3::KeyFrame* add_candidate(unsigned long id, ORB_SLAM3::Map& map,
                                     double shared_weight, bool bad = false)
    {
        candidates.push_back(std::make_unique<TestKeyFrame>(id, map, shared_weight, bad));
        ORB_SLAM3::KeyFrame* candidate = candidates.back().get();
        database.add(candidate);
        return candidate;
    }
};

void test_bad_candidate_before_valid(const std::shared_ptr<spdlog::logger>& logger)
{
    Fixture fixture(logger);
    fixture.add_candidate(1, fixture.current_map, 0.95, true);
    ORB_SLAM3::KeyFrame* loop = fixture.add_candidate(2, fixture.current_map, 0.8);
    ORB_SLAM3::KeyFrame* merge = fixture.add_candidate(3, fixture.merge_map, 0.6);

    std::vector<ORB_SLAM3::KeyFrame*> loops;
    std::vector<ORB_SLAM3::KeyFrame*> merges;
    fixture.database.DetectNBestCandidates(&fixture.query, loops, merges, 1);
    require(loops == std::vector<ORB_SLAM3::KeyFrame*>{loop},
            "Bad leading candidate prevented selection of the valid loop candidate");
    require(merges == std::vector<ORB_SLAM3::KeyFrame*>{merge},
            "Bad leading candidate prevented selection of the valid merge candidate");
}

void test_all_candidates_bad(const std::shared_ptr<spdlog::logger>& logger)
{
    Fixture fixture(logger);
    fixture.add_candidate(1, fixture.current_map, 0.9, true);
    fixture.add_candidate(2, fixture.merge_map, 0.7, true);

    std::vector<ORB_SLAM3::KeyFrame*> loops;
    std::vector<ORB_SLAM3::KeyFrame*> merges;
    fixture.database.DetectNBestCandidates(&fixture.query, loops, merges, 3);
    require(loops.empty() && merges.empty(), "Bad candidates must not be returned");
}

void test_bad_candidates_between_and_after_valid(
    const std::shared_ptr<spdlog::logger>& logger)
{
    Fixture fixture(logger);
    ORB_SLAM3::KeyFrame* loop = fixture.add_candidate(1, fixture.current_map, 0.9);
    fixture.add_candidate(2, fixture.merge_map, 0.8, true);
    ORB_SLAM3::KeyFrame* merge = fixture.add_candidate(3, fixture.merge_map, 0.7);
    fixture.add_candidate(4, fixture.current_map, 0.6, true);

    std::vector<ORB_SLAM3::KeyFrame*> loops;
    std::vector<ORB_SLAM3::KeyFrame*> merges;
    // Neither result reaches its limit, so detection must also pass the bad tail.
    fixture.database.DetectNBestCandidates(&fixture.query, loops, merges, 3);
    require(loops == std::vector<ORB_SLAM3::KeyFrame*>{loop},
            "Skipping bad candidates changed the valid loop result");
    require(merges == std::vector<ORB_SLAM3::KeyFrame*>{merge},
            "Bad middle candidate prevented selection of the valid merge candidate");
}

void test_candidate_limits_and_order(const std::shared_ptr<spdlog::logger>& logger)
{
    Fixture fixture(logger);
    // Deliberately add candidates out of score order to exercise real sorting.
    fixture.add_candidate(1, fixture.current_map, 0.5);
    ORB_SLAM3::KeyFrame* second_merge = fixture.add_candidate(2, fixture.merge_map, 0.6);
    ORB_SLAM3::KeyFrame* first_loop = fixture.add_candidate(3, fixture.current_map, 0.9);
    fixture.add_candidate(4, fixture.merge_map, 0.4);
    ORB_SLAM3::KeyFrame* first_merge = fixture.add_candidate(5, fixture.merge_map, 0.8);
    ORB_SLAM3::KeyFrame* second_loop = fixture.add_candidate(6, fixture.current_map, 0.7);

    std::vector<ORB_SLAM3::KeyFrame*> loops;
    std::vector<ORB_SLAM3::KeyFrame*> merges;
    fixture.database.DetectNBestCandidates(&fixture.query, loops, merges, 2);
    require(loops == std::vector<ORB_SLAM3::KeyFrame*>{first_loop, second_loop},
            "Loop candidate score order or count limit changed");
    require(merges == std::vector<ORB_SLAM3::KeyFrame*>{first_merge, second_merge},
            "Merge candidate score order or count limit changed");
}
} // namespace

int main()
{
    try
    {
        const auto logger = std::make_shared<spdlog::logger>(
            "keyframe_candidates_test", std::make_shared<spdlog::sinks::null_sink_mt>());
        test_bad_candidate_before_valid(logger);
        test_all_candidates_bad(logger);
        test_bad_candidates_between_and_after_valid(logger);
        test_candidate_limits_and_order(logger);
        std::cout << "Keyframe database candidate tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Keyframe database candidate test failed: " << error.what() << '\n';
        return 1;
    }
}
