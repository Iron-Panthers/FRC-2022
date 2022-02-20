package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class ReplaceCamelCaseTests {
  private ReplaceCamelCase replaceCamelCase;

  @BeforeEach
  public void setup() {
    replaceCamelCase = new ReplaceCamelCase();
  }

  public static Stream<Arguments> camelCaseSentenceProvider() {
    return Stream.of(
        Arguments.of(
            "aaaaa", "aaaaa ", "because there must be an ending space to leave a gap for ()"),
        Arguments.of(
            "helloWorld",
            "hello world ",
            "because capitals should be lowercase and given a space prior"),
        Arguments.of(
            "HelloWorld",
            "hello world ",
            "because first capital should be ignored, not given a space"),
        Arguments.of(
            "talonFX",
            "talon FX ",
            "because multiple capitalized letters should be treated as a word"),
        Arguments.of(
            "TalonFXTest",
            "talon FX test ",
            "because next word should be broken off after capital section"));
  }

  @UtilParamTest
  @MethodSource("camelCaseSentenceProvider")
  public void camelCaseIsModifiedProperly(String camelCase, String sentence, String explanation) {
    assertEquals(
        sentence,
        replaceCamelCase.replaceCamelCase(camelCase),
        String.format("replaceCamelCase(\"%s\") == \"%s\" %s", camelCase, sentence, explanation));
  }
}