package frc;

import java.lang.reflect.Method;
import org.junit.jupiter.api.DisplayNameGenerator;

public class ReplaceCamelCase extends DisplayNameGenerator.Standard {
  @Override
  public String generateDisplayNameForClass(Class<?> testClass) {
    return replaceCamelCase(super.generateDisplayNameForClass(testClass));
  }

  @Override
  public String generateDisplayNameForNestedClass(Class<?> nestedClass) {
    return replaceCamelCase(super.generateDisplayNameForNestedClass(nestedClass));
  }

  @Override
  public String generateDisplayNameForMethod(Class<?> testClass, Method testMethod) {
    return this.replaceCamelCase(testMethod.getName())
        + DisplayNameGenerator.parameterTypesAsString(testMethod);
  }

  String replaceCamelCase(String camelCase) {
    StringBuilder result = new StringBuilder();
    for (char c : camelCase.toCharArray()) {
      if (Character.isUpperCase(c)) {
        result.append(' ');
        result.append(Character.toLowerCase(c));
      } else {
        result.append(c);
      }
    }
    result.append(' ');
    return result.toString();
  }
}
